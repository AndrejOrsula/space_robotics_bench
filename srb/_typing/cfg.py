from __future__ import annotations

from typing import TYPE_CHECKING, Any, Union

if TYPE_CHECKING:
    from srb.core.env import BaseEnvCfg, DirectEnv, ManagedEnv

    AnyEnv = Union[DirectEnv, ManagedEnv]
    # Direct, managed, and MARL configs all derive from BaseEnvCfg.
    AnyEnvCfg = BaseEnvCfg
else:
    # Pydantic/configclass skips Any validation; object causes strict errors.
    AnyEnv = Any
    AnyEnvCfg = Any
