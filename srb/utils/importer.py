import importlib
import os
import pkgutil
import sys
from dataclasses import dataclass
from typing import Iterable, Sequence

from srb.utils import logging


@dataclass(frozen=True)
class RecursiveImportFailure:
    module_name: str
    error: Exception


def import_recursively(
    module_name: str, ignorelist: Sequence[str] = (), modules_only: bool = True
) -> tuple[RecursiveImportFailure, ...]:
    """Import modules beneath ``module_name`` and return logged failures.

    The package itself must import successfully. Descendants matching the sweep
    are imported unless ignored or already loaded.
    """

    try:
        package = importlib.import_module(module_name)
    except ImportError as e:
        logging.critical(f"Failed to import base package '{module_name}'")
        raise e

    package_path = getattr(package, "__path__", None)
    if not package_path:
        return ()

    failures: list[RecursiveImportFailure] = []
    for _ in _import_recursively_impl(
        path=package_path,
        prefix=f"{package.__name__}.",
        ignorelist=ignorelist,
        modules_only=modules_only,
        failures=failures,
    ):
        pass

    if failures:
        logging.critical(_format_import_failure_summary(module_name, failures))
    return tuple(failures)


def _format_import_failure_summary(
    module_name: str,
    failures: Sequence[RecursiveImportFailure],
    *,
    limit: int = 8,
) -> str:
    shown = failures[:limit]
    details = "; ".join(f"{failure.module_name}: {failure.error}" for failure in shown)
    remaining = len(failures) - len(shown)
    suffix = f"; ... {remaining} more" if remaining else ""
    package_word = "package" if len(failures) == 1 else "packages"
    return (
        f"{len(failures)} {package_word} failed while recursively importing "
        f"'{module_name}': {details}{suffix}"
    )


def _import_recursively_impl(
    path: Iterable[str],
    prefix: str,
    ignorelist: Sequence[str],
    modules_only: bool,
    failures: list[RecursiveImportFailure],
    seen_paths: set[str] | None = None,
) -> Iterable:
    path = tuple(path)
    if seen_paths is None:
        seen_paths = set(path)

    if modules_only:
        for info in pkgutil.iter_modules(path, prefix):
            if any(module_name in info.name for module_name in ignorelist):
                continue

            yield info

            try:
                module = importlib.import_module(info.name)
            except Exception as e:
                failures.append(RecursiveImportFailure(info.name, e))
                continue

            if info.ispkg:
                sub_paths = getattr(module, "__path__", None) or []
                unseen_paths = [p for p in sub_paths if p not in seen_paths]
                if unseen_paths:
                    seen_paths.update(unseen_paths)
                    yield from _import_recursively_impl(
                        unseen_paths,
                        f"{info.name}.",
                        ignorelist,
                        modules_only,
                        failures,
                        seen_paths,
                    )
    else:
        for base_path in path:
            for root, dirs, files in os.walk(base_path):
                # Ignore bytecode and data files; only Python modules are importable.
                dirs[:] = [name for name in dirs if name != "__pycache__"]
                for filename in files:
                    if not filename.endswith(".py"):
                        continue
                    full_path = os.path.join(root, filename)
                    rel_path = os.path.relpath(full_path, base_path)

                    module_rel_path, _ = os.path.splitext(rel_path)
                    module_name_to_import = prefix + module_rel_path.replace(
                        os.sep, "."
                    )

                    if any(ignored in module_name_to_import for ignored in ignorelist):
                        continue

                    if module_name_to_import in sys.modules:
                        continue

                    try:
                        __import__(module_name_to_import)
                        yield module_name_to_import
                    except Exception as e:
                        # Missing dependencies can leave an adapter out of the registry.
                        failures.append(
                            RecursiveImportFailure(module_name_to_import, e)
                        )
