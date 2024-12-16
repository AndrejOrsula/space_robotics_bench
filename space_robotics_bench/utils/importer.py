import importlib
import pkgutil
import sys
from typing import Iterable, List


def import_modules_recursively(module_name: str, ignorelist: List[str] = []):
    package = importlib.import_module(module_name)
    for _ in _walk_modules(
        path=package.__path__, prefix=f"{package.__name__}.", ignorelist=ignorelist
    ):
        pass


def _walk_modules(
    path: Iterable[str],
    prefix: str = "",
    ignorelist: List[str] = [],
):
    def seen(p, m={}):
        if p in m:
            return True
        m[p] = True

    for info in pkgutil.iter_modules(path, prefix):
        if any(module_name in info.name for module_name in ignorelist):
            continue

        yield info

        if info.ispkg:
            try:
                __import__(info.name)
            except Exception:
                raise
            else:
                paths = getattr(sys.modules[info.name], "__path__", None) or []
                paths = [path for path in paths if not seen(path)]
                yield from _walk_modules(paths, f"{info.name}.", ignorelist)
