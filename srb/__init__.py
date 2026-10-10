from importlib.metadata import PackageNotFoundError, version

from srb.composition import (
    install_application_bindings as _install_application_bindings,
)

try:
    __version__ = version("srb")
except PackageNotFoundError:
    __version__ = "0.0.0-unknown"

_install_application_bindings()
del _install_application_bindings
