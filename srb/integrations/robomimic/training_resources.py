"""Manage datasets, loggers, and workers owned by a Robomimic training run."""

from __future__ import annotations

import shutil
import time
from collections.abc import Callable, Iterable, Iterator, Mapping
from contextlib import contextmanager
from dataclasses import dataclass
from multiprocessing.context import BaseContext
from pathlib import Path
from types import TracebackType
from typing import Any, Literal, TypeVar

import torch
from torch.utils.data import DataLoader
from torch.utils.data.dataloader import _MultiProcessingDataLoaderIter

from srb.utils.failures import attempt_cleanup

T = TypeVar("T")
_WORKER_JOIN_TIMEOUT_SECONDS = 1.0


class RobomimicTrainingCleanupError(RuntimeError):
    """Cleanup failed; the retained owner can retry it."""

    def __init__(
        self,
        message: str,
        *,
        owner: Any,
        errors: tuple[BaseException, ...],
    ) -> None:
        self.owner = owner
        self.cleanup_owner = owner
        self.errors = errors
        detail = "; ".join(f"{type(error).__name__}: {error}" for error in errors)
        super().__init__(f"{message}: {detail}" if detail else message)

    def retry_cleanup(self) -> None:
        """Retry cleanup through the owner that still retains its resources."""

        self.owner.retry_cleanup()


def _flatten_cleanup_errors(error: BaseException) -> tuple[BaseException, ...]:
    if not isinstance(error, RobomimicTrainingCleanupError):
        return (error,)
    flattened: list[BaseException] = []
    for nested in error.errors:
        flattened.extend(_flatten_cleanup_errors(nested))
    return tuple(flattened)


@dataclass
class _ResourceEntry:
    resource: Any
    close: Callable[[], object]


class _ConstructionScope:
    """Track processes and queues created during iterator construction."""

    def __init__(self) -> None:
        self.processes: list[Any] = []
        self.queues: list[Any] = []

    def record_process(self, process: Any) -> None:
        self.processes.append(process)

    def record_queue(self, queue: Any) -> None:
        self.queues.append(queue)

    def close(self) -> tuple[BaseException, ...]:
        errors: list[BaseException] = []
        deadline = time.monotonic() + _WORKER_JOIN_TIMEOUT_SECONDS
        for process in tuple(self.processes):
            pid = getattr(process, "pid", None)
            if pid is None:
                continue
            is_alive = getattr(process, "is_alive", None)
            alive = False
            if callable(is_alive):
                result, error = attempt_cleanup(is_alive)
                if error is not None:
                    errors.append(error)
                    continue
                alive = bool(result)
            if alive:
                terminate = getattr(process, "terminate", None)
                if callable(terminate):
                    _, error = attempt_cleanup(terminate)
                    if error is not None:
                        errors.append(error)
            join = getattr(process, "join", None)
            if callable(join):
                _, error = attempt_cleanup(
                    lambda: join(timeout=max(0.0, deadline - time.monotonic()))
                )
                if error is not None:
                    errors.append(error)
                    continue
            if callable(is_alive):
                result, error = attempt_cleanup(is_alive)
                if error is not None:
                    errors.append(error)
                elif result:
                    errors.append(
                        RuntimeError(
                            "Robomimic data-loader construction worker remained alive"
                        )
                    )

        if errors:
            return tuple(errors)
        return _close_queue_objects(self.queues)


class _TrackingContext(BaseContext):
    """Record processes and queues created for one loader."""

    def __init__(self, base: BaseContext, loader: RobomimicDataLoader) -> None:
        self._base = base
        self._loader = loader
        self._name = base.get_start_method()

    def Process(self, *args: Any, **kwargs: Any) -> Any:
        process = getattr(self._base, "Process")(*args, **kwargs)
        scope = self._loader._construction_scope
        if scope is not None:
            scope.record_process(process)
        return process

    def Queue(self, *args: Any, **kwargs: Any) -> Any:
        queue = getattr(self._base, "Queue")(*args, **kwargs)
        scope = self._loader._construction_scope
        if scope is not None:
            scope.record_queue(queue)
        return queue

    def __getattr__(self, name: str) -> Any:
        return getattr(self._base, name)


def _close_queue_objects(queues: Iterable[Any]) -> tuple[BaseException, ...]:
    errors: list[BaseException] = []
    seen: set[int] = set()
    for queue in queues:
        identity = id(queue)
        if identity in seen:
            continue
        seen.add(identity)
        cancel_join_thread = getattr(queue, "cancel_join_thread", None)
        if callable(cancel_join_thread):
            _, error = attempt_cleanup(cancel_join_thread)
            if error is not None:
                errors.append(error)
        close = getattr(queue, "close", None)
        if callable(close):
            _, error = attempt_cleanup(close)
            if error is not None:
                errors.append(error)
    return tuple(errors)


class _TrackedIterator(Iterator[Any]):
    """Close the provider iterator when iteration ends."""

    def __init__(self, loader: RobomimicDataLoader, source: Any) -> None:
        self._loader = loader
        self._source = source

    def __iter__(self) -> _TrackedIterator:
        return self

    def __next__(self) -> Any:
        try:
            return next(self._source)
        except StopIteration:
            self._loader._finish_iterator(self._source)
            raise


class _SafeMultiProcessingDataLoaderIter(_MultiProcessingDataLoaderIter):
    """Guard provider cleanup when iterator startup fails partway through."""

    def __del__(self) -> None:
        if not hasattr(self, "_workers_status"):
            return
        super().__del__()  # type: ignore[no-untyped-call]


class RobomimicDataLoader(DataLoader[Any]):
    """PyTorch loader with explicit iterator cleanup.

    Arguments pass through unchanged. ``close_iterators`` closes the current
    epoch's iterators; persistent workers restart next epoch. ``close`` also
    prevents future iteration.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._active_iterators: list[Any] = []
        self._partial_constructions: list[_ConstructionScope] = []
        self._construction_scope: _ConstructionScope | None = None
        self._tracking_context: _TrackingContext | None = None
        if self.num_workers > 0:
            base_context = self.multiprocessing_context
            if base_context is None:
                base_context = torch.multiprocessing.get_context()
            self._tracking_context = _TrackingContext(base_context, self)
        self._closed = False

    @property
    def closed(self) -> bool:
        """Whether final cleanup completed."""

        return self._closed

    def __iter__(self) -> Any:
        if self._closed:
            raise RuntimeError("Robomimic data loader is closed")
        scope = _ConstructionScope()
        original_context = self.multiprocessing_context
        self._construction_scope = scope
        if self._tracking_context is not None:
            self.multiprocessing_context = self._tracking_context
        constructed = False
        try:
            source = super().__iter__()
            constructed = True
        finally:
            if not constructed:
                self._partial_constructions.append(scope)
            self._construction_scope = None
            self.multiprocessing_context = original_context
        if source is None:
            raise RuntimeError(
                "PyTorch data-loader iterator construction returned None"
            )
        if not any(existing is source for existing in self._active_iterators):
            self._active_iterators.append(source)
        return _TrackedIterator(self, source)

    def _get_iterator(self) -> Any:
        if self.num_workers == 0:
            return super()._get_iterator()
        self.check_worker_number_rationality()  # type: ignore[no-untyped-call]
        return _SafeMultiProcessingDataLoaderIter(  # type: ignore[no-untyped-call]
            self
        )

    def _finish_iterator(self, source: Any) -> None:
        self._close_iterator(source)

    def _close_iterator(self, source: Any) -> None:
        shutdown_error: BaseException | None = None
        shutdown = getattr(source, "_shutdown_workers", None)
        if callable(shutdown):
            result, error = attempt_cleanup(shutdown)
            if error is not None:
                shutdown_error = error
            elif result is False:
                shutdown_error = RuntimeError(
                    "PyTorch worker shutdown remained incomplete"
                )
        else:
            close = getattr(source, "close", None)
            if callable(close):
                result, error = attempt_cleanup(close)
                if error is not None:
                    shutdown_error = error
                elif result is False:
                    shutdown_error = RuntimeError(
                        "data-loader iterator close remained incomplete"
                    )
        worker_error = self._wait_for_workers(source)
        queue_errors = (
            self._close_queues(source)
            if worker_error is None and callable(shutdown)
            else ()
        )
        errors: list[BaseException] = []
        if shutdown_error is not None:
            errors.extend(_flatten_cleanup_errors(shutdown_error))
        if worker_error is not None:
            errors.extend(_flatten_cleanup_errors(worker_error))
        for queue_error in queue_errors:
            errors.extend(_flatten_cleanup_errors(queue_error))
        if errors:
            raise RobomimicTrainingCleanupError(
                "PyTorch worker shutdown remained incomplete",
                owner=self,
                errors=tuple(errors),
            ) from errors[0]
        self._active_iterators = [
            active for active in self._active_iterators if active is not source
        ]
        if self.persistent_workers and getattr(self, "_iterator", None) is source:
            self._iterator = None

    @staticmethod
    def _wait_for_workers(source: Any) -> BaseException | None:
        workers = getattr(source, "_workers", ()) or ()
        deadline = time.monotonic() + _WORKER_JOIN_TIMEOUT_SECONDS
        errors: list[BaseException] = []
        for worker in tuple(workers):
            join = getattr(worker, "join", None)
            if callable(join):
                _, error = attempt_cleanup(
                    lambda: join(timeout=max(0.0, deadline - time.monotonic()))
                )
                if error is not None:
                    errors.append(error)
                    continue
            is_alive = getattr(worker, "is_alive", None)
            if callable(is_alive):
                result, error = attempt_cleanup(is_alive)
                if error is not None:
                    errors.append(error)
                    continue
                if result:
                    errors.append(
                        RuntimeError("PyTorch data-loader worker remained alive")
                    )
            elif callable(join):
                errors.append(
                    RuntimeError("PyTorch data-loader worker has no liveness check")
                )
        if not errors:
            return None
        if len(errors) == 1:
            return errors[0]
        return RobomimicTrainingCleanupError(
            "PyTorch data-loader workers did not finish",
            owner=source,
            errors=tuple(errors),
        )

    @staticmethod
    def _close_queues(source: Any) -> tuple[BaseException, ...]:
        queues: list[Any] = []
        for name in ("_index_queues", "_worker_result_queue", "_data_queue"):
            candidate = getattr(source, name, None)
            if isinstance(candidate, (list, tuple)):
                queues.extend(candidate)
            elif candidate is not None:
                queues.append(candidate)
        return _close_queue_objects(queues)

    def close_iterators(self) -> None:
        """Close retained iterators."""

        if self._closed:
            return
        errors: list[BaseException] = []
        for scope in tuple(self._partial_constructions):
            scope_result, scope_error = attempt_cleanup(scope.close)
            if scope_error is not None:
                errors.append(scope_error)
                continue
            scope_errors = scope_result if isinstance(scope_result, tuple) else ()
            if scope_errors:
                errors.extend(scope_errors)
            else:
                self._partial_constructions = [
                    pending
                    for pending in self._partial_constructions
                    if pending is not scope
                ]
        for source in tuple(self._active_iterators):
            _, iterator_error = attempt_cleanup(lambda: self._close_iterator(source))
            if iterator_error is not None:
                errors.extend(_flatten_cleanup_errors(iterator_error))
        if errors:
            raise RobomimicTrainingCleanupError(
                "Robomimic data-loader iterator cleanup failed",
                owner=self,
                errors=tuple(errors),
            ) from errors[0]

    def retry_cleanup(self) -> None:
        """Retry iterator and worker cleanup."""

        self.close_iterators()

    def close(self) -> None:
        """Clean up iterators and close this loader."""

        if self._closed:
            return
        self.close_iterators()
        self._closed = True


def _dataset_children(dataset: Any) -> tuple[Any, ...] | None:
    children = getattr(dataset, "datasets", None)
    if children is None:
        return None
    if isinstance(children, Mapping):
        return tuple(children.values())
    try:
        return tuple(children)
    except TypeError as error:
        raise TypeError("MetaDataset.datasets must be iterable") from error


def _dataset_leaves(dataset: Any, seen: set[int] | None = None) -> Iterator[Any]:
    if seen is None:
        seen = set()
    identity = id(dataset)
    if identity in seen:
        return
    seen.add(identity)

    children = _dataset_children(dataset)
    if children:
        yielded = False
        for child in children:
            for leaf in _dataset_leaves(child, seen):
                yielded = True
                yield leaf
        if yielded:
            return

    close = getattr(dataset, "close_and_delete_hdf5_handle", None)
    if not callable(close):
        raise TypeError(
            "Robomimic dataset must expose close_and_delete_hdf5_handle "
            "or iterable datasets"
        )
    yield dataset


def _remove_temporary(path: Path) -> None:
    if path.is_symlink():
        raise RuntimeError("normalization temporary must be a real directory")
    if path.exists():
        shutil.rmtree(path)


class RobomimicTrainingResources:
    """Own and close resources for one training run."""

    def __init__(self) -> None:
        self._loaders: list[_ResourceEntry] = []
        self._datasets: list[_ResourceEntry] = []
        self._loggers: list[_ResourceEntry] = []
        self._inputs: list[_ResourceEntry] = []
        self._temporaries: list[_ResourceEntry] = []
        self._closed = False

    @property
    def closed(self) -> bool:
        """Whether all registered resources are closed."""

        return self._closed

    def __enter__(self) -> RobomimicTrainingResources:
        self._ensure_open()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> Literal[False]:
        if exc_value is None:
            self.close()
            return False
        try:
            self.close()
        except RobomimicTrainingCleanupError as cleanup_error:
            errors = (exc_value, *cleanup_error.errors)
            raise RobomimicTrainingCleanupError(
                "Robomimic training failed and cleanup remains incomplete",
                owner=self,
                errors=errors,
            ) from exc_value
        return False

    def own_input(self, owner: T) -> T:
        """Keep an input context alive until dependent resources close."""

        self._ensure_open()
        close = getattr(owner, "close", None)
        if not callable(close):
            raise TypeError("training input context must expose close()")
        self._add(self._inputs, owner, close)
        return owner

    def own_dataset(self, dataset: T) -> T:
        """Register dataset leaves, including ``MetaDataset`` children."""

        self._ensure_open()
        for leaf in _dataset_leaves(dataset):
            close = getattr(leaf, "close_and_delete_hdf5_handle", None)
            if not callable(close):
                raise TypeError(
                    "Robomimic dataset must expose close_and_delete_hdf5_handle()"
                )
            self._add(self._datasets, leaf, close)
        return dataset

    def own_logger(self, logger: T) -> T:
        """Register a logger and any external log file it owns."""

        self._ensure_open()
        owned = False
        close = getattr(logger, "close", None)
        if callable(close):
            self._add(self._loggers, logger, close)
            owned = True

        log_file = getattr(logger, "log_file", None)
        file_close = getattr(log_file, "close", None)
        if callable(file_close) and log_file is not logger:
            self._add(self._loggers, log_file, file_close)
            owned = True

        if not owned:
            raise TypeError("training logger must expose close() or log_file.close()")
        return logger

    def own_temporary(self, temporary: str | Path) -> Path:
        """Remove the normalization directory after inputs close."""

        self._ensure_open()
        path = Path(temporary)
        self._add(self._temporaries, path, lambda: _remove_temporary(path))
        return path

    def data_loader(self, *args: Any, **kwargs: Any) -> RobomimicDataLoader:
        """Create and register a cleanup-aware PyTorch loader."""

        self._ensure_open()
        loader = RobomimicDataLoader(*args, **kwargs)
        self._add(self._loaders, loader, loader.close)
        return loader

    def close_iterators(self) -> None:
        """Close loader workers after a provider ``run_epoch`` call."""

        if self._closed:
            return
        errors: list[BaseException] = []
        for entry in tuple(self._loaders):
            loader = entry.resource
            close_iterators = getattr(loader, "close_iterators")
            result, cleanup_error = attempt_cleanup(close_iterators)
            if cleanup_error is None and result is False:
                cleanup_error = RuntimeError(
                    "data-loader iterator cleanup remained incomplete"
                )
            if cleanup_error is not None:
                errors.extend(_flatten_cleanup_errors(cleanup_error))
        if errors:
            raise RobomimicTrainingCleanupError(
                "Robomimic data-loader iterator cleanup failed",
                owner=self,
                errors=tuple(errors),
            ) from errors[0]

    @contextmanager
    def epoch_iterators(self) -> Iterator[None]:
        """Close loader iterators after one epoch without masking its failure."""

        try:
            yield
        except BaseException as exc:
            try:
                self.close_iterators()
            except RobomimicTrainingCleanupError as cleanup_error:
                raise RobomimicTrainingCleanupError(
                    "Robomimic epoch failed and iterator cleanup failed",
                    owner=self,
                    errors=(exc, *cleanup_error.errors),
                ) from exc
            raise
        self.close_iterators()

    def retry_cleanup(self) -> None:
        """Retry cleanup in dependency order."""

        self.close()

    def close(self) -> None:
        """Close workers, datasets, loggers, inputs, and temporary files."""

        if self._closed:
            return

        errors = self._attempt(self._loaders, "close")
        if errors:
            self._raise_cleanup("Robomimic worker cleanup failed", errors)

        dataset_errors = self._attempt(self._datasets, "close")
        logger_errors = self._attempt(self._loggers, "close")
        errors = [*dataset_errors, *logger_errors]
        if errors:
            self._raise_cleanup("Robomimic provider cleanup failed", errors)

        input_errors = self._attempt(self._inputs, "close")
        if input_errors:
            self._raise_cleanup("Robomimic input cleanup failed", input_errors)

        temporary_errors = self._attempt(self._temporaries, "close")
        if temporary_errors:
            self._raise_cleanup(
                "Robomimic normalization temporary cleanup failed",
                temporary_errors,
            )

        self._closed = True

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("Robomimic training resources are closed")

    @staticmethod
    def _add(
        entries: list[_ResourceEntry],
        resource: Any,
        close: Callable[[], object],
    ) -> None:
        if any(entry.resource is resource for entry in entries):
            return
        entries.append(_ResourceEntry(resource=resource, close=close))

    @staticmethod
    def _attempt(
        entries: list[_ResourceEntry],
        operation: str,
    ) -> list[BaseException]:
        errors: list[BaseException] = []
        for entry in tuple(entries):
            result, error = attempt_cleanup(entry.close)
            if error is None and result is False:
                error = RuntimeError(f"resource {operation} remained incomplete")
            if error is not None:
                errors.extend(_flatten_cleanup_errors(error))
            else:
                entries[:] = [
                    candidate for candidate in entries if candidate is not entry
                ]
        return errors

    def _raise_cleanup(self, message: str, errors: list[BaseException]) -> None:
        raise RobomimicTrainingCleanupError(
            message,
            owner=self,
            errors=tuple(errors),
        ) from errors[0]


__all__ = [
    "RobomimicDataLoader",
    "RobomimicTrainingCleanupError",
    "RobomimicTrainingResources",
]
