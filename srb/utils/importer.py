import importlib
import os
import pkgutil
import sys
from typing import Iterable, Sequence

from srb.utils import logging


def import_recursively(
    module_name: str, ignorelist: Sequence[str] = (), modules_only: bool = True
):
    try:
        package = importlib.import_module(module_name)
    except ImportError as e:
        logging.critical(f"Failed to import base package '{module_name}'")
        raise e

    package_path = getattr(package, "__path__", None)
    if not package_path:
        return

    for _ in _import_recursively_impl(
        path=package_path,
        prefix=f"{package.__name__}.",
        ignorelist=ignorelist,
        modules_only=modules_only,
    ):
        pass


def _import_recursively_impl(
    path: Iterable[str],
    prefix: str,
    ignorelist: Sequence[str],
    modules_only: bool,
) -> Iterable:
    if modules_only:

        def __seen(p, m={}):
            if p in m:
                return True
            m[p] = True
            return False

        for info in pkgutil.iter_modules(path, prefix):
            if any(module_name in info.name for module_name in ignorelist):
                continue

            yield info

            if info.ispkg:
                try:
                    module = importlib.import_module(info.name)
                except Exception as e:
                    logging.critical(f"Failed to import '{info.name}'")
                    raise e
                else:
                    sub_paths = getattr(module, "__path__", None) or []
                    unseen_paths = [p for p in sub_paths if not __seen(p)]
                    if unseen_paths:
                        yield from _import_recursively_impl(
                            unseen_paths, f"{info.name}.", ignorelist, modules_only
                        )
    else:
        for base_path in path:
            for root, _, files in os.walk(base_path):
                for filename in files:
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
                    except (ImportError, ModuleNotFoundError):
                        pass
                    except Exception as e:
                        logging.warning(
                            f"Failed to import file as module '{module_name_to_import}': {e}"
                        )
