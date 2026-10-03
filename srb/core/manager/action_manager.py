from collections.abc import Iterator
from typing import Callable, TypeVar

from isaaclab.managers import ActionManager as __ActionManager
from isaaclab.managers import ActionTerm, ActionTermCfg

from srb.utils._action_term_lookup import get_term_or_none, require_term
from srb.utils.config_introspection import iter_config_items

ActionTermT = TypeVar("ActionTermT", bound=ActionTerm)


class ActionManager(__ActionManager):
    def iter_terms(self) -> Iterator[tuple[str, ActionTerm]]:
        """Iterate action terms in manager order."""
        return iter(self._terms.items())

    def iter_terms_by_type(
        self, term_type: type[ActionTermT]
    ) -> Iterator[tuple[str, ActionTermT]]:
        """Iterate action terms of the given type."""
        for term_name, term in self.iter_terms():
            if isinstance(term, term_type):
                yield term_name, term

    def find_term_by_type(
        self, term_type: type[ActionTermT]
    ) -> tuple[str, ActionTermT] | None:
        """Return the first action term of the given type."""
        return next(self.iter_terms_by_type(term_type), None)

    def get_term_by_type(self, term_type: type[ActionTermT]) -> ActionTermT | None:
        """Return the first action term of the given type, or ``None``."""
        return get_term_or_none(self.find_term_by_type(term_type))

    def require_term_by_type(
        self,
        term_type: type[ActionTermT],
        *,
        reason: str,
    ) -> ActionTermT:
        """Return the first action term of the given type or raise ``ValueError``.

        ``reason`` explains why the term is required.
        """
        return require_term(
            self.find_term_by_type(term_type),
            cls=term_type,
            reason=reason,
        )

    def _prepare_terms(self):
        self._term_names: list[str] = []
        self._terms: dict[str, ActionTerm] = {}

        if isinstance(self.cfg, dict):
            cfg_items = self.cfg.items()
        else:
            cfg_items = iter_config_items(self.cfg)
        for term_name, term_cfg in cfg_items:
            if term_cfg is None or isinstance(term_cfg, Callable):
                continue
            if not isinstance(term_cfg, ActionTermCfg):
                raise TypeError(
                    f"Configuration for the term '{term_name}' is not of type ActionTermCfg."
                    f" Received: '{type(term_cfg)}'."
                )
            term = term_cfg.class_type(term_cfg, self._env)
            if not isinstance(term, ActionTerm):
                raise TypeError(
                    f"Returned object for the term '{term_name}' is not of type ActionTerm."
                )
            self._term_names.append(term_name)
            self._terms[term_name] = term
