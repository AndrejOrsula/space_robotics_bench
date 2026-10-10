"""Monkey-patch for upstream DreamerV3 replay selector.

The upstream ``embodied.replay.selectors.Mixture`` does not expose a
``__len__`` method, which SRB's replay-buffer introspection (used by
checkpoint/resume logic) relies on. This module provides a thin subclass
that delegates ``__len__`` to the first underlying selector, and is the
selector class imported throughout the SRB Dreamer integration instead
of ``embodied.replay.selectors.Mixture`` directly.

If ``embodied`` adds ``__len__`` upstream, this shim can be removed.
"""

import embodied


class Mixture(embodied.replay.selectors.Mixture):
    """Drop-in replacement for ``embodied.replay.selectors.Mixture`` with ``__len__``."""

    def __len__(self):
        return len(self.selectors[0])
