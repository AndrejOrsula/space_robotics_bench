"""Spec metadata shared by the task-space action groups.

Kept free of action-group classes so both group modules can import it without
registering (or re-registering) any ``ActionGroup``.
"""

from srb.core.spec import TermSpec

#: Channel order of a relative task-space pose command: a position delta
#: followed by an axis-angle rotation delta, both in the articulation root frame.
POSE_DELTA_LAYOUT = ("dx", "dy", "dz", "rx", "ry", "rz")
POSE_DELTA_UNITS = "m,m,m,rad,rad,rad (axis-angle)"

#: Channel layout shared by the operational-space groups. ``pose_rel`` uses the
#: first six channels; ``variable_kp`` appends six stiffness channels and
#: ``variable`` appends six stiffness and then six damping-ratio channels.
OSC_DELTA_TWIST_LAYOUT = (
    *POSE_DELTA_LAYOUT,
    "stiffness[6] (variable_kp, variable)",
    "damping_ratio[6] (variable)",
)

#: The operational-space groups own one runtime term, ``delta_twist``. (1,) is
#: the placeholder the builder resolves to the action manager's width (6, 12
#: or 18 by impedance mode).
OSC_DELTA_TWIST_TERM = TermSpec(
    name="delta_twist",
    shape=(1,),
    dtype="float32",
    units=POSE_DELTA_UNITS + "; impedance channels are normalized gains",
    frame="root",
    semantic_label="end_effector.delta_pose_with_impedance",
)
