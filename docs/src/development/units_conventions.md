# Units & Conventions

## Unit System

Use Isaac Sim/Isaac Lab SI units. `sunlight_rpy` and domain solar angular
diameters may use degrees.

| Quantity | SI unit |
|---|---|
| Length | m |
| Mass | kg |
| Time | s |
| Angle | rad |
| Force | N |
| Torque | N·m |
| Gravity | m/s² |

## Coordinate Frame

Use **right-handed, Z-up** coordinates: X forward, Y left, Z up; gravity is
negative Z.

## Quaternion Format

SRB and Isaac Lab use **(x, y, z, w)**; USD/OpenUSD `Gf.Quatf` and some
external libraries use **(w, x, y, z)**, converted by SRB.

- `srb.core.static_asset_view.XformPrimView.get_world_poses` returns and `set_world_poses` accepts
  (x, y, z, w); the setter converts to USD.
- Sensor mount `OffsetCfg.rot` and event-camera `rot_xyzw` use it unchanged.

## Frame Suffix Convention

Use these frame suffixes:

Use `_w` for absolute world frames (`root_pos_w`, `body_quat_w`) and `_b` for
relative body/base frames (`ee_vel_b`).

## Domain-Specific Gravity

| Domain | Gravity (m/s²) |
|---|---|
| Earth | 9.80665 |
| Mars | 3.72076 |
| Moon | 1.62496 |
| Asteroid | 0.14219 |
| Orbit | 0.0 |

## Spec Authoring (`@spec` decorator)

Apply `@spec` from `srb.core.spec` to concrete `ActionGroup`, SRB-native
`SensorBase`, and observation-manager classes. Import registers
`ComponentDescriptor`; the environment aggregates them into queryable
`env.srb_spec` (`EnvSpec`), separate from Gymnasium `env.spec`.

### Two authoring styles

Use `fields=` for up to five fields and `spec_field` otherwise. Never combine.

**Explicit fields:**

```python
from srb.core.spec import spec, TermSpec

@spec(kind="action", fields=[
    TermSpec(name="lin", shape=(1,), dtype="float32",
             units="m/s", min=-2.0, max=2.0),
    TermSpec(name="ang", shape=(1,), dtype="float32",
             units="rad/s", min=-1.0, max=1.0),
])
class WheeledDriveActionGroup(ActionGroup):
    ...
```

**`spec_field` attributes (long schemas):**

```python
from srb.core.spec import spec, spec_field

@spec(kind="action")
class SmoothOSCActionGroup(ActionGroup):
    ee_pos = spec_field("ee_pos", dtype="float32", shape=(3,), units="m")
    ee_rot = spec_field("ee_rot", dtype="float32", shape=(4,), units="quat")
    stiffness = spec_field("stiffness", dtype="float32", shape=(6,),
                           units="N/m, Nm/rad")
```

### What `kind=` to choose

- `"action"` for `ActionGroup`; `"sensor"` for `SensorBase`.
- `"observation"` for managers; set extra `group="policy"` to scope.

### Privileged terms

Mark deployed-policy exclusions `privileged=True`, such as ground-truth
particle positions:

```python
TermSpec(name="particles_xyz", shape=(N, 3), dtype="float32",
         privileged=True,
         semantic_label="excavation.particle_positions")
```

`projections.onnx_export_mask(spec.observation)` returns privileged or
fallback-provenance term names; ONNX export ignores the mask.

### Registration requirement

Register concrete subclasses with `@spec(...)`.

### Versioned action metadata

Action descriptors are serialized contracts. Increment the schema version if
term names, axes, normalization, scaling, frame, or units change.

| Descriptor | Contract |
|---|---|
| Multicopter `cmd_vel` (v2) | Axes `linear_x`, `linear_y`, `linear_z`, `angular_z` are normalized and clamped to `[-1, 1]` before scaling by `MulticopterBodyAccelerationActionCfg.scale`. The three linear channels use `m/s`; angular z uses `rad/s`. Body x/y rotate by yaw only (ignore roll/pitch); z stays world vertical, and angular z is yaw rate. |
| Orbital body `body_acc` mode (`ISS`, `Gateway`) | Descriptor `body_vel` (v2). |
| Inverse-kinematics and operational-space groups | One `delta_twist`; width is 3 position, 3 axis-angle rotation, plus control-mode impedance channels. |

For descriptor versions, runtime widths, and rejected signatures, see the
[Action Space Catalog](../reference/action_spaces.md).
