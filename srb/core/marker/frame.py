from srb.core.marker import FRAME_MARKER_CFG as _FRAME_MARKER_CFG_ORIG

# Use a smaller debug scale than Isaac Lab's 0.5 default.
FRAME_MARKER_CFG = _FRAME_MARKER_CFG_ORIG.copy()  # type: ignore[misc]
FRAME_MARKER_CFG.markers["frame"].scale = (0.1, 0.1, 0.1)

FRAME_MARKER_SMALL_CFG = _FRAME_MARKER_CFG_ORIG.copy()  # type: ignore[misc]
FRAME_MARKER_SMALL_CFG.markers["frame"].scale = (0.025, 0.025, 0.025)
