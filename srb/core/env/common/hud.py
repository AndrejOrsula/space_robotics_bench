"""Lightweight in-sim HUD helpers.

The HUD is intentionally optional and lazy: importing SRB in a headless or
non-Omniverse process must not require ``omni.ui``. Runtime failures disable
the overlay instead of breaking the environment.
"""

from __future__ import annotations

from collections.abc import Sequence

from srb.utils import logging

HudLine = tuple[str, str]


class DemoHud:
    """Small Omniverse UI window for live task metrics."""

    def __init__(
        self,
        *,
        title: str,
        max_lines: int = 8,
        width: int = 300,
        height: int | None = None,
    ) -> None:
        self._enabled = False
        self._ui = None
        self._window = None
        self._labels = []
        self._max_lines = max(1, max_lines)

        try:
            import omni.ui as ui  # type: ignore[import-not-found]
        except Exception as e:
            logging.debug(f"HUD disabled because omni.ui is unavailable: {e}")
            return

        self._ui = ui
        try:
            self._window = ui.Window(
                title,
                width=width,
                height=height or (42 + 30 * self._max_lines),
                visible=True,
            )
            with self._window.frame:
                with ui.VStack(
                    spacing=6,
                    style={
                        "background_color": 0xD0181A1E,
                        "border_color": 0x44FFFFFF,
                        "border_width": 1,
                        "border_radius": 6,
                        "padding": 12,
                    },
                ):
                    self._labels = [
                        ui.Label(
                            "",
                            word_wrap=False,
                            style={
                                "color": 0xFFF2F5F7 if i == 0 else 0xFFD5D1C7,
                                "font_size": 26 if i == 0 else 15,
                            },
                        )
                        for i in range(self._max_lines)
                    ]
        except Exception as e:
            logging.warning(f"Failed to create HUD window: {e}")
            self._window = None
            self._labels = []
            return

        self._enabled = True

    @property
    def enabled(self) -> bool:
        return self._enabled

    def update(self, lines: Sequence[HudLine]) -> None:
        if not self._enabled:
            return

        try:
            for idx, label in enumerate(self._labels):
                if idx < len(lines):
                    key, value = lines[idx]
                    label.text = f"{key}: {value}" if key else value
                else:
                    label.text = ""
        except Exception as e:
            logging.warning(f"Disabling HUD after update failure: {e}")
            self.close()

    def close(self) -> None:
        self._enabled = False
        try:
            if self._window is not None:
                self._window.visible = False
        except Exception:
            pass
        finally:
            self._window = None
            self._labels = []
