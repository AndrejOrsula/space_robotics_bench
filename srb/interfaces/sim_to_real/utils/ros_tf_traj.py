#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from functools import partial
from threading import Thread
from typing import TYPE_CHECKING, Literal

import numpy
from pydantic import BaseModel

if TYPE_CHECKING:
    from geometry_msgs.msg import Transform
    from rclpy.node import Node as RosNode
    from rclpy.timer import Timer


@dataclass
class PatternState:
    current_pos: numpy.ndarray
    current_quat_wxyz: numpy.ndarray
    is_done: bool = False


@dataclass
class LinePatternState(PatternState):
    dist_traveled: float = 0.0
    path_direction: int = 1


@dataclass
class RectanglePatternState(PatternState):
    segment_idx: int = 0
    dist_on_segment: float = 0.0


@dataclass
class CirclePatternState(PatternState):
    angle: float = 0.0


@dataclass
class LemniscatePatternState(PatternState):
    t: float = 0.0


@dataclass
class LissajousPatternState(PatternState):
    t: float = 0.0


@dataclass
class CapsulePatternState(PatternState):
    segment_idx: int = 0
    dist_on_segment: float = 0.0


@dataclass
class SpiralPatternState(PatternState):
    phase: float = 0.0


class BasePatternCfg(BaseModel):
    initial_pos: numpy.ndarray = numpy.array((0.0, 0.0, 0.0), dtype=numpy.float32)
    initial_quat_wxyz: numpy.ndarray = numpy.array(
        (1.0, 0.0, 0.0, 0.0), dtype=numpy.float32
    )

    direction: Literal["clockwise", "counter-clockwise"] = "counter-clockwise"

    class Config:
        arbitrary_types_allowed = True


class LinePatternCfg(BasePatternCfg):
    length: float = 1.0
    direction: None = None


class RectanglePatternCfg(BasePatternCfg):
    width: float = 1.0
    height: float = 1.0


class CirclePatternCfg(BasePatternCfg):
    radius: float = 0.5


class LemniscatePatternCfg(BasePatternCfg):
    scale: float = 1.0


class LissajousPatternCfg(BasePatternCfg):
    scale: float = 1.0


class CapsulePatternCfg(BasePatternCfg):
    length: float = 1.0
    radius: float = 0.25


class SpiralPatternCfg(BasePatternCfg):
    max_radius: float = 1.0
    n_loops: int = 3


class RosTfTrajectoryGeneratorCfg(BaseModel):
    pattern: BasePatternCfg
    parent_frame_id: str = "world"
    child_frame_id: str = "target"
    rate: float = 50.0
    velocity: float = 1.0
    n_loops: int = 1


def quat_to_rot_mat(q_wxyz: numpy.ndarray) -> numpy.ndarray:
    w, x, y, z = q_wxyz
    return numpy.array(
        [
            [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
        ]
    )


def multiply_quats(q1_wxyz: numpy.ndarray, q2_wxyz: numpy.ndarray) -> numpy.ndarray:
    w1, x1, y1, z1 = q1_wxyz
    w2, x2, y2, z2 = q2_wxyz
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return numpy.array([w, x, y, z])


def yaw_to_quat_wxyz(yaw: float, deg: bool = False) -> numpy.ndarray:
    if deg:
        yaw = math.radians(yaw)
    return numpy.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])


def _get_transform_msg(pos: numpy.ndarray, quat_wxyz: numpy.ndarray) -> "Transform":
    from geometry_msgs.msg import Quaternion, Transform, Vector3

    return Transform(
        translation=Vector3(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])),
        rotation=Quaternion(
            w=float(quat_wxyz[0]),
            x=float(quat_wxyz[1]),
            y=float(quat_wxyz[2]),
            z=float(quat_wxyz[3]),
        ),
    )


class RosTfTrajectoryGenerator:
    def __init__(
        self,
        cfg: RosTfTrajectoryGeneratorCfg,
        ros_node: "RosNode | None" = None,
        start_thread: bool = True,
    ):
        from geometry_msgs.msg import TransformStamped
        from std_msgs.msg import Header
        from tf2_ros import TransformBroadcaster

        self.cfg = cfg
        self.pattern_cfg = cfg.pattern
        self.n_loops, self.is_running = cfg.n_loops, False

        self._velocity_sign = numpy.sign(cfg.velocity)
        self._abs_step_size = abs(cfg.velocity / cfg.rate)
        self._reverse_yaw_flip = math.pi if self._velocity_sign < 0 else 0

        self._initial_rot_mat = quat_to_rot_mat(self.pattern_cfg.initial_quat_wxyz)
        self._pattern_state = self._initialize_state(self.pattern_cfg)

        if isinstance(self.pattern_cfg, CapsulePatternCfg):
            self._cap_segment_lengths = [
                self.pattern_cfg.length,
                math.pi * self.pattern_cfg.radius,
            ] * 2
        elif isinstance(self.pattern_cfg, LemniscatePatternCfg):
            self._initialize_lemniscate_compensation()
        elif isinstance(self.pattern_cfg, LissajousPatternCfg):
            self._initialize_lissajous_compensation()

        if not ros_node:
            import rclpy
            from rclpy.node import Node as RosNode

            if not rclpy.ok():
                rclpy.init()
            self._ros_node = RosNode("ros_tf_trajectory_broadcaster")
            self._owns_node = True
        else:
            self._ros_node, self._owns_node = ros_node, False

        self.tf_broadcaster = TransformBroadcaster(self._ros_node)
        self.msg = TransformStamped(
            header=Header(frame_id=cfg.parent_frame_id),
            child_frame_id=cfg.child_frame_id,
        )
        self._timer: "Timer | None" = None
        self._thread: "Thread | None" = None
        if start_thread:
            self.start()

    def _initialize_state(self, p_cfg: BasePatternCfg) -> PatternState:
        # The start position is ALWAYS the initial_pos from the config.
        args = {
            "current_pos": p_cfg.initial_pos.copy(),
            "current_quat_wxyz": p_cfg.initial_quat_wxyz.copy(),
        }

        if isinstance(p_cfg, LinePatternCfg):
            return LinePatternState(**args)  # type: ignore
        if isinstance(p_cfg, RectanglePatternCfg):
            return RectanglePatternState(**args)  # type: ignore
        if isinstance(p_cfg, CirclePatternCfg):
            return CirclePatternState(**args)  # type: ignore
        if isinstance(p_cfg, LemniscatePatternCfg):
            return LemniscatePatternState(**args)  # type: ignore
        if isinstance(p_cfg, LissajousPatternCfg):
            return LissajousPatternState(**args)  # type: ignore
        if isinstance(p_cfg, CapsulePatternCfg):
            return CapsulePatternState(**args)  # type: ignore
        if isinstance(p_cfg, SpiralPatternCfg):
            # If velocity is negative, start at the end of the path (phase=2.0)
            initial_phase = 2.0 if self._velocity_sign < 0 else 0.0
            args["phase"] = initial_phase  # type: ignore
            return SpiralPatternState(**args)  # type: ignore
        raise TypeError(f"Unknown pattern config type: {type(p_cfg)}")

    def _update_transform(self):
        state, p_cfg = self._pattern_state, self.pattern_cfg
        state.is_done = False
        if isinstance(p_cfg, LinePatternCfg):
            self._update_line(p_cfg, state)  # type: ignore
        elif isinstance(p_cfg, RectanglePatternCfg):
            self._update_rectangle(p_cfg, state)  # type: ignore
        elif isinstance(p_cfg, CirclePatternCfg):
            self._update_circle(p_cfg, state)  # type: ignore
        elif isinstance(p_cfg, LemniscatePatternCfg):
            self._update_lemniscate(p_cfg, state)  # type: ignore
        elif isinstance(p_cfg, LissajousPatternCfg):
            self._update_lissajous(p_cfg, state)  # type: ignore
        elif isinstance(p_cfg, CapsulePatternCfg):
            self._update_capsule(p_cfg, state)  # type: ignore
        elif isinstance(p_cfg, SpiralPatternCfg):
            self._update_spiral(p_cfg, state)  # type: ignore
        return _get_transform_msg(state.current_pos, state.current_quat_wxyz)

    def _update_line(self, cfg: LinePatternCfg, state: LinePatternState):
        # Update distance along the current leg of the path.
        # Velocity sign does not affect the path traversal, only the final orientation.
        state.dist_traveled += state.path_direction * self._abs_step_size

        # Check for bounce conditions
        if state.dist_traveled >= cfg.length:
            state.dist_traveled = cfg.length
            state.path_direction = -1
        elif state.dist_traveled <= 0:
            state.dist_traveled = 0
            state.path_direction = 1
            state.is_done = True  # A full back-and-forth trip is complete

        local_offset = numpy.array([state.dist_traveled, 0, 0])
        # The direction of the path's tangent
        local_yaw = 0 if state.path_direction > 0 else math.pi

        state.current_pos = cfg.initial_pos + (
            self._velocity_sign * self._initial_rot_mat @ local_offset
        )
        # Apply the reverse flip to the tangent's orientation
        state.current_quat_wxyz = multiply_quats(
            cfg.initial_quat_wxyz, yaw_to_quat_wxyz(local_yaw)
        )

    def _update_rectangle(self, cfg: RectanglePatternCfg, state: RectanglePatternState):
        w, h, dir_sign = (
            cfg.width,
            cfg.height,
            1 if cfg.direction == "counter-clockwise" else -1,
        )
        offsets = [
            numpy.array([0, 0]),
            numpy.array([w, 0]),
            numpy.array([w, h * dir_sign]),
            numpy.array([0, h * dir_sign]),
        ]
        dirs = [
            numpy.array([1, 0]),
            numpy.array([0, dir_sign]),
            numpy.array([-1, 0]),
            numpy.array([0, -dir_sign]),
        ]
        yaws = [0, dir_sign * math.pi / 2, math.pi, dir_sign * 3 * math.pi / 2]

        # Update position along the current segment
        state.dist_on_segment += self._abs_step_size * self._velocity_sign

        # Handle moving to the next or previous segment
        if state.segment_idx % 2 == 0:  # Horizontal segments
            segment_length = w
        else:  # Vertical segments
            segment_length = h

        if state.dist_on_segment >= segment_length:
            state.dist_on_segment -= segment_length
            state.segment_idx = (state.segment_idx + 1) % 4
            if state.segment_idx == 0:
                state.is_done = True
        elif state.dist_on_segment < 0:
            state.segment_idx = (state.segment_idx - 1) % 4
            state.dist_on_segment += segment_length
            if state.segment_idx == 3:
                state.is_done = True

        local_offset = (
            offsets[state.segment_idx] + dirs[state.segment_idx] * state.dist_on_segment
        )
        local_yaw = yaws[state.segment_idx]

        state.current_pos = cfg.initial_pos + self._initial_rot_mat @ numpy.array(
            [
                local_offset[0],
                local_offset[1],
                0,
            ]
        )
        state.current_quat_wxyz = multiply_quats(
            cfg.initial_quat_wxyz, yaw_to_quat_wxyz(local_yaw)
        )

    def _update_circle(self, cfg: CirclePatternCfg, state: CirclePatternState):
        # state.angle is the absolute distance traveled / radius. Always increases.
        state.angle += self._abs_step_size / (cfg.radius if cfg.radius > 0 else 1)
        if state.angle >= 2 * math.pi:
            state.angle = 0.0
            state.is_done = True

        # The parameter 'a' is the angle used in the parameterization. Its sign depends on velocity.
        a = state.angle * self._velocity_sign

        if cfg.direction == "counter-clockwise":
            # CCW Path starts at (0,0) tangent to +X: P(a) = (r*sin(a), r*(1-cos(a)))
            # The tangent's yaw is 'a'.
            local_offset = numpy.array(
                [
                    cfg.radius * math.sin(a),
                    cfg.radius * (1 - math.cos(a)),
                    0,
                ]
            )
            local_yaw = a
        else:  # Clockwise
            # CW Path starts at (0,0) tangent to +X: P(a) = (r*sin(a), -r*(1-cos(a)))
            # The tangent's yaw is '-a'.
            local_offset = numpy.array(
                [
                    cfg.radius * math.sin(a),
                    -cfg.radius * (1 - math.cos(a)),
                    0,
                ]
            )
            local_yaw = -a

        # Transform local path to world frame using initial pose
        state.current_pos = cfg.initial_pos + self._initial_rot_mat @ local_offset
        # Combine initial orientation with the path's tangent orientation (and reverse flip)
        state.current_quat_wxyz = multiply_quats(
            cfg.initial_quat_wxyz, yaw_to_quat_wxyz(local_yaw)
        )

    def _initialize_lemniscate_compensation(self):
        dir_sign = -1.0 if self.pattern_cfg.direction == "counter-clockwise" else 1.0
        # The tangent of the lemniscate at t=pi/2 (the center) is (-1, -1/2).
        # We need to rotate the whole pattern to align this with the +X axis.
        self._inf_lemniscate_start_yaw = math.atan2(-0.5 * dir_sign, -0.5)
        angle = -self._inf_lemniscate_start_yaw
        c, s = math.cos(angle), math.sin(angle)
        self._inf_lemniscate_compensation_rot = numpy.array([[c, -s], [s, c]])

    def _update_lemniscate(
        self, cfg: LemniscatePatternCfg, state: LemniscatePatternState
    ):
        dir_sign = -1.0 if cfg.direction == "counter-clockwise" else 1.0

        def get_raw_lemniscate(t: float):
            cos_t, sin_t = math.cos(t), math.sin(t)
            den = 1 + sin_t**2
            if den == 0:
                return numpy.array([0, 0, 0])
            x = cfg.scale * cos_t / den
            y = cfg.scale * dir_sign * sin_t * cos_t / den
            return numpy.array([x, y, 0])

        t_start = math.pi / 2
        t_current = state.t + t_start

        eps = 1e-6
        pos_t1_raw = get_raw_lemniscate(t_current)
        pos_t2_raw = get_raw_lemniscate(t_current + eps * self._velocity_sign)
        tangent_raw = pos_t2_raw - pos_t1_raw

        pos_t1_comp = self._inf_lemniscate_compensation_rot @ pos_t1_raw[:2]
        pos_t2_comp = self._inf_lemniscate_compensation_rot @ pos_t2_raw[:2]
        inst_speed = numpy.linalg.norm(pos_t2_comp - pos_t1_comp) / eps

        if inst_speed > 1e-5:
            state.t += self._abs_step_size * self._velocity_sign / inst_speed

        if abs(state.t) >= 2 * math.pi:
            state.t, state.is_done = 0.0, True

        t_final = state.t + t_start
        local_offset_raw = get_raw_lemniscate(t_final)

        compensated_offset_2d = (
            self._inf_lemniscate_compensation_rot @ local_offset_raw[:2]
        )

        raw_yaw = math.atan2(tangent_raw[1], tangent_raw[0])
        compensated_yaw = raw_yaw - self._inf_lemniscate_start_yaw

        state.current_pos = cfg.initial_pos + self._initial_rot_mat @ numpy.array(
            [
                compensated_offset_2d[0],
                compensated_offset_2d[1],
                0,
            ]
        )
        state.current_quat_wxyz = multiply_quats(
            cfg.initial_quat_wxyz,
            yaw_to_quat_wxyz(compensated_yaw + self._reverse_yaw_flip),
        )

    def _initialize_lissajous_compensation(self):
        dir_sign = 1.0 if self.pattern_cfg.direction == "counter-clockwise" else -1.0
        # The tangent of (sin(2t), dir*sin(t)) at t=0 is (2, dir).
        self._inf_intrinsic_start_yaw = math.atan2(dir_sign, 2.0)
        # Create a 2D rotation matrix to cancel out this initial yaw
        c = math.cos(-self._inf_intrinsic_start_yaw)
        s = math.sin(-self._inf_intrinsic_start_yaw)
        self._inf_compensation_rot = numpy.array([[c, -s], [s, c]])

    def _update_lissajous(self, cfg: LissajousPatternCfg, state: LissajousPatternState):
        dir_sign = 1.0 if cfg.direction == "counter-clockwise" else -1.0

        def get_local_pos_at_t(t):
            # Lissajous curve x=sin(2t), y=sin(t)
            return numpy.array(
                [
                    cfg.scale * math.sin(2 * t),
                    cfg.scale * dir_sign * math.sin(t),
                    0,
                ]
            )

        eps = 1e-6
        tangent_vec_uncompensated = get_local_pos_at_t(
            state.t + eps * self._velocity_sign
        ) - get_local_pos_at_t(state.t)
        inst_speed = numpy.linalg.norm(tangent_vec_uncompensated) / eps
        if inst_speed > 1e-5:
            state.t += self._abs_step_size * self._velocity_sign / inst_speed

        if abs(state.t) >= 2 * math.pi:
            state.t, state.is_done = 0.0, True

        local_offset_uncompensated = get_local_pos_at_t(state.t)

        # Rotate the entire pattern so that its starting tangent is along the X-axis
        compensated_offset_2d = (
            self._inf_compensation_rot @ local_offset_uncompensated[:2]
        )

        state.current_pos = cfg.initial_pos + self._initial_rot_mat @ numpy.array(
            [
                compensated_offset_2d[0],
                compensated_offset_2d[1],
                0,
            ]
        )

        # Calculate the yaw of the tangent on the uncompensated curve
        tangent_yaw_uncompensated = math.atan2(
            tangent_vec_uncompensated[1], tangent_vec_uncompensated[0]
        )

        # The yaw in the local frame is the tangent's yaw minus the initial compensation angle
        local_yaw = tangent_yaw_uncompensated - self._inf_intrinsic_start_yaw

        state.current_quat_wxyz = multiply_quats(
            cfg.initial_quat_wxyz, yaw_to_quat_wxyz(local_yaw + self._reverse_yaw_flip)
        )

    def _update_capsule(self, cfg: CapsulePatternCfg, state: CapsulePatternState):
        length, radius, dir_sign = (
            cfg.length,
            cfg.radius,
            1 if cfg.direction == "counter-clockwise" else -1,
        )

        # Update position along the current segment
        state.dist_on_segment += self._abs_step_size * self._velocity_sign

        # Handle moving to the next or previous segment
        if state.dist_on_segment >= self._cap_segment_lengths[state.segment_idx]:
            state.dist_on_segment -= self._cap_segment_lengths[state.segment_idx]
            state.segment_idx = (state.segment_idx + 1) % 4
            if state.segment_idx == 0:
                state.is_done = True
        elif state.dist_on_segment < 0:
            state.segment_idx = (state.segment_idx - 1) % 4
            state.dist_on_segment += self._cap_segment_lengths[state.segment_idx]
            if state.segment_idx == 3:
                state.is_done = True

        dist = state.dist_on_segment
        if state.segment_idx == 0:  # First straight line
            offset, yaw = numpy.array([dist, 0, 0]), 0
        elif state.segment_idx == 1:  # First arc
            phi = dist / radius
            center = numpy.array([length, dir_sign * radius, 0])
            angle = dir_sign * phi - (math.pi / 2 * dir_sign)
            offset = center + numpy.array(
                [
                    radius * math.cos(angle),
                    radius * math.sin(angle),
                    0,
                ]
            )
            yaw = dir_sign * phi
        elif state.segment_idx == 2:  # Second straight line
            offset, yaw = (
                numpy.array([length - dist, 2 * radius * dir_sign, 0]),
                math.pi,
            )
        else:  # state.segment_idx == 3, Second arc
            phi = dist / radius
            center = numpy.array([0, dir_sign * radius, 0])
            angle = dir_sign * phi + (math.pi / 2 * dir_sign)
            offset = center + numpy.array(
                [
                    radius * math.cos(angle),
                    radius * math.sin(angle),
                    0,
                ]
            )
            yaw = math.pi + dir_sign * phi

        state.current_pos = cfg.initial_pos + self._initial_rot_mat @ offset
        state.current_quat_wxyz = multiply_quats(
            cfg.initial_quat_wxyz, yaw_to_quat_wxyz(yaw)
        )

    def _update_spiral(self, cfg: SpiralPatternCfg, state: SpiralPatternState):
        p = state.phase
        N = cfg.n_loops
        R = cfg.max_radius
        dir_sign = 1 if cfg.direction == "counter-clockwise" else -1

        # This parameterization creates a two-part trajectory (out and in)
        # controlled by a single phase variable from 0.0 to 2.0.
        # - Phase 0.0 -> 1.0: Spiral out. Radius increases linearly with phase.
        # - Phase 1.0 -> 2.0: Spiral in. Radius decreases linearly.
        # - Angle increases continuously throughout, ensuring no reversal of direction.
        C = dir_sign * 2 * math.pi * N  # Angular constant

        def get_derivatives_at_phase(phase_val):
            # Use a small epsilon to avoid singularity at the center
            phase_val = max(phase_val, 1e-6)

            # Radius (r) and its derivative (dr/dp) change based on the leg
            if 0.0 <= phase_val < 1.0:
                r = R * phase_val
                dr_dp = R
            else:  # 1.0 <= phase_val <= 2.0
                r = R * (2.0 - phase_val)
                dr_dp = -R

            # Angle (theta) and its derivative (dtheta/dp) are continuous
            theta = C * phase_val
            dtheta_dp = C

            # Calculate tangent vector (dx/dp, dy/dp) using the chain rule
            cos_t, sin_t = math.cos(theta), math.sin(theta)
            dx_dp = dr_dp * cos_t - r * dtheta_dp * sin_t
            dy_dp = dr_dp * sin_t + r * dtheta_dp * cos_t

            return dx_dp, dy_dp

        # --- Update phase based on constant velocity ---
        dx_dp, dy_dp = get_derivatives_at_phase(p)
        inst_speed = math.sqrt(dx_dp**2 + dy_dp**2)

        if inst_speed > 1e-5:
            # Phase increases or decreases based on velocity sign
            state.phase += self._abs_step_size * self._velocity_sign / inst_speed

        # Check for completion of the full out-and-in trajectory from either direction
        if self._velocity_sign > 0 and state.phase >= 2.0:
            state.phase = 0.0  # Reset for next loop
            state.is_done = True
        elif self._velocity_sign < 0 and state.phase <= 0.0:
            state.phase = 2.0  # Reset for next loop
            state.is_done = True

        # --- Calculate final pose for the newly updated phase ---
        p_final = numpy.clip(state.phase, 0.0, 2.0)

        # Calculate final radius and theta based on the appropriate leg
        if 0.0 <= p_final < 1.0:
            r_final = R * p_final
        else:
            r_final = R * (2.0 - p_final)
        theta_final = C * p_final

        # Position in the local (pattern) frame
        local_offset = numpy.array(
            [
                r_final * math.cos(theta_final),
                r_final * math.sin(theta_final),
                0,
            ]
        )

        # Recalculate tangent at the new position to determine orientation
        dx_dp_final, dy_dp_final = get_derivatives_at_phase(p_final)
        local_yaw = math.atan2(dy_dp_final, dx_dp_final)

        # Transform local path pose to the world frame using the initial pose
        state.current_pos = cfg.initial_pos + self._initial_rot_mat @ local_offset
        # Apply orientation flips for velocity direction
        state.current_quat_wxyz = multiply_quats(
            cfg.initial_quat_wxyz, yaw_to_quat_wxyz(local_yaw)
        )

    def start(self):
        if self.is_running:
            return
        self.is_running = True
        if self.cfg.n_loops != 0:
            self._timer = self._ros_node.create_timer(1.0 / self.cfg.rate, self.cb)
        if self._owns_node and not self._thread:
            import rclpy

            self._thread = Thread(
                target=partial(rclpy.spin, self._ros_node), daemon=True
            )
            self._thread.start()

    def stop(self):
        if not self.is_running:
            return
        if self._timer and not self._timer.is_canceled():
            self._timer.cancel()
        self.is_running = False
        self._ros_node.get_logger().info("Trajectory broadcaster stopped.")

    def cb(self):
        if not self.is_running:
            return

        self.msg.header.stamp = self._ros_node.get_clock().now().to_msg()
        self.msg.transform = self._update_transform()
        self.tf_broadcaster.sendTransform(self.msg)
        if self._pattern_state.is_done:
            if self.n_loops > 0:
                self.n_loops -= 1
            if self.n_loops == 0:
                self._ros_node.get_logger().info("Finished all loops.")
                self.stop()

    def shutdown(self):
        import rclpy

        self.stop()
        if self._owns_node:
            self._ros_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        if self._thread:
            self._thread.join()


def main():
    print("Initializing trajectory broadcaster...")

    pattern_cfg = CapsulePatternCfg(
        length=3.0,
        radius=1.15,
        direction="counter-clockwise",
        initial_pos=numpy.array([0.0, 0.0, 1.0], dtype=numpy.float32),
        initial_quat_wxyz=yaw_to_quat_wxyz(0.0, deg=True),
    )
    # pattern_cfg = LemniscatePatternCfg(
    #     scale=2.65,
    #     direction="clockwise",
    #     initial_pos=numpy.array([3.6, -3.1, 0.0], dtype=numpy.float32),
    #     initial_quat_wxyz=yaw_to_quat_wxyz(45.0, deg=True),
    # )
    # pattern_cfg = LissajousPatternCfg(
    #     scale=1.5,
    #     direction="counter-clockwise",
    #     initial_pos=numpy.array([2.75, -2.9, 0.0], dtype=numpy.float32),
    #     initial_quat_wxyz=yaw_to_quat_wxyz(-155.0, deg=True),
    # )
    # pattern_cfg = RectanglePatternCfg(
    #     width=4.3,
    #     height=2.5,
    #     direction="clockwise",
    #     initial_pos=numpy.array([1.1, -1.5, 0.0], dtype=numpy.float32),
    #     initial_quat_wxyz=yaw_to_quat_wxyz(0.0, deg=True),
    # )
    # pattern_cfg = CirclePatternCfg(
    #     radius=1.4,
    #     direction="counter-clockwise",
    #     initial_pos=numpy.array([2.7, -4.15, 0.0], dtype=numpy.float32),
    #     initial_quat_wxyz=yaw_to_quat_wxyz(0.0, deg=True),
    # )
    # pattern_cfg = SpiralPatternCfg(
    #     max_radius=1.0,
    #     n_loops=2,
    #     direction="clockwise",
    #     initial_pos=numpy.array([2.0, -2.75, 0.0], dtype=numpy.float32),
    #     initial_quat_wxyz=yaw_to_quat_wxyz(0.0, deg=True),
    # )
    # pattern_cfg = LinePatternCfg(
    #     length=4.5,
    #     initial_pos=numpy.array([1.0, -2.75, 0.0], dtype=numpy.float32),
    #     initial_quat_wxyz=yaw_to_quat_wxyz(0.0, deg=True),
    # )

    # Set n_loops to -1 for infinite looping.
    # Set velocity to a negative value to run the trajectory backward.
    traj_cfg = RosTfTrajectoryGeneratorCfg(
        pattern=pattern_cfg,
        n_loops=-1,
        velocity=0.02,
        rate=20.0,
    )

    broadcaster = None
    try:
        broadcaster = RosTfTrajectoryGenerator(cfg=traj_cfg)
        print(
            f"Broadcasting TF '{traj_cfg.child_frame_id}' relative to '{traj_cfg.parent_frame_id}'. Press Ctrl+C to stop."
        )
        while broadcaster.is_running:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nShutdown requested by user.")
    except (ImportError, ModuleNotFoundError):
        print(
            "\nError: Could not import ROS2 libraries. Please ensure ROS2 is installed and sourced."
        )
    finally:
        if broadcaster:
            print("Shutting down...")
            broadcaster.shutdown()
            print("Shutdown complete.")


if __name__ == "__main__":
    main()
