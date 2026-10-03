"""Tagged randomization terms, queries, and runtime event-manager helpers.

Use ``SrbEventTermCfg`` with an explicit category for new terms. Existing
``EventTermCfg`` fields remain visible as ``UNTAGGED`` through
``RandomizationView`` and can be migrated gradually. The view filters active
terms by category or trigger, accepting enum values, names, and string values.

``active()`` reports configured terms. ``live_event_terms`` reports terms
still executable after manager materialization and runtime removals.
``live_event_term_names_by_mode`` returns detached names grouped by manager
mode, preserving empty groups.
"""

from srb.core.randomization.runtime import (
    live_event_term_names_by_mode,
    live_event_terms,
    remove_event_terms_by_name_fragments,
)
from srb.core.randomization.term import (
    RandomizationCategory,
    RandomizationTrigger,
    SrbEventTermCfg,
)
from srb.core.randomization.view import RandomizationView

__all__ = [
    "RandomizationCategory",
    "RandomizationTrigger",
    "RandomizationView",
    "SrbEventTermCfg",
    "live_event_term_names_by_mode",
    "live_event_terms",
    "remove_event_terms_by_name_fragments",
]
