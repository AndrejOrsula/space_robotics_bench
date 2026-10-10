#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal

import numpy as np
from pydantic import BaseModel, Field

from srb.interfaces._ros_node import acquire_ros_node_runtime

if TYPE_CHECKING:
    from geometry_msgs.msg import Transform
    from rclpy.node import Node as RosNode
    from rclpy.timer import Timer


@dataclass
class PatternState:
    current_pos: np.ndarray
    current_quat: np.ndarray
    is_done: bool = False


@dataclass
class LinePatternState(PatternState):
    dist_traveled: float = 0.0
    path_direction: int = 1


@dataclass
class RectanglePatternState(PatternState):
    segment_idx: int = 0
    dist_on_segment: float = 0.0
    distance_traveled: float = 0.0


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
    distance_traveled: float = 0.0


@dataclass
class SpiralPatternState(PatternState):
    phase: float = 0.0


class BasePatternCfg(BaseModel):
    initial_pos: np.ndarray = np.array((0.0, 0.0, 0.0), dtype=np.float32)
    initial_quat: np.ndarray = np.array((0.0, 0.0, 0.0, 1.0), dtype=np.float32)

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
    # Arc updates divide by the radius.
    radius: float = Field(default=0.25, gt=0.0)


class SpiralPatternCfg(BasePatternCfg):
    max_radius: float = 1.0
    n_loops: int = 3


class RosTfTrajectoryGeneratorCfg(BaseModel):
    pattern: BasePatternCfg
    parent_frame_id: str = "world"
    child_frame_id: str = "target"
    rate: float = Field(default=50.0, gt=0.0, allow_inf_nan=False)
    velocity: float = Field(default=1.0, allow_inf_nan=False)
    n_loops: int = 1


def quat_to_rot_mat(quat: np.ndarray) -> np.ndarray:
    x, y, z, w = quat
    return np.array(
        [
            [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y],
        ]
    )


def multiply_quats(quat1: np.ndarray, quat2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = quat1
    x2, y2, z2, w2 = quat2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return np.array([x, y, z, w])


def yaw_to_quat(yaw: float, deg: bool = False) -> np.ndarray:
    if deg:
        yaw = math.radians(yaw)
    # xyzw (w-last), matching initial_quat, multiply_quats, and quat_to_rot_mat
    # in this module. A yaw rotation about +z is (x=0, y=0, z=sin, w=cos).
    return np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])


def _get_transform_msg(pos: np.ndarray, quat: np.ndarray) -> "Transform":
    from geometry_msgs.msg import Quaternion, Transform, Vector3

    return Transform(
        translation=Vector3(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])),
        # quat is xyzw (w-last) per this module's convention; map to the ROS
        # geometry_msgs Quaternion (x, y, z, w) without reordering the slots.
        rotation=Quaternion(
            x=float(quat[0]),
            y=float(quat[1]),
            z=float(quat[2]),
            w=float(quat[3]),
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

        self._velocity_sign = np.sign(cfg.velocity)
        self._abs_step_size = abs(cfg.velocity / cfg.rate)
        self._reverse_yaw_flip = math.pi if self._velocity_sign < 0 else 0

        self._initial_rot_mat = quat_to_rot_mat(self.pattern_cfg.initial_quat)
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

        self._ros_runtime = acquire_ros_node_runtime(
            ros_node, name="ros_tf_trajectory_broadcaster", namespace=""
        )
        self._ros_node = self._ros_runtime.node

        self.tf_broadcaster = TransformBroadcaster(self._ros_node)
        self.msg = TransformStamped(
            header=Header(frame_id=cfg.parent_frame_id),
            child_frame_id=cfg.child_frame_id,
        )
        self._timer: "Timer | None" = None
        if start_thread:
            self.start()

    def _initialize_state(self, p_cfg: BasePatternCfg) -> PatternState:
        args = {
            "current_pos": p_cfg.initial_pos.copy(),
            "current_quat": p_cfg.initial_quat.copy(),
        }

        if isinstance(p_cfg, LinePatternCfg):
            return LinePatternState(**args)  # type: ignore[return-value]
        if isinstance(p_cfg, RectanglePatternCfg):
            return RectanglePatternState(**args)  # type: ignore[return-value]
        if isinstance(p_cfg, CirclePatternCfg):
            return CirclePatternState(**args)  # type: ignore[return-value]
        if isinstance(p_cfg, LemniscatePatternCfg):
            return LemniscatePatternState(**args)  # type: ignore[return-value]
        if isinstance(p_cfg, LissajousPatternCfg):
            return LissajousPatternState(**args)  # type: ignore[return-value]
        if isinstance(p_cfg, CapsulePatternCfg):
            return CapsulePatternState(**args)  # type: ignore[return-value]
        if isinstance(p_cfg, SpiralPatternCfg):
            # Reverse playback starts from the trajectory's terminal phase.
            initial_phase = 2.0 if self._velocity_sign < 0 else 0.0
            args["phase"] = initial_phase  # type: ignore[misc]
            return SpiralPatternState(**args)  # type: ignore[return-value]
        raise TypeError(f"Unknown pattern config type: {type(p_cfg)}")

    def _update_transform(self):
        state, p_cfg = self._pattern_state, self.pattern_cfg
        state.is_done = False
        if isinstance(p_cfg, LinePatternCfg):
            self._update_line(p_cfg, state)  # type: ignore[misc]
        elif isinstance(p_cfg, RectanglePatternCfg):
            self._update_rectangle(p_cfg, state)  # type: ignore[misc]
        elif isinstance(p_cfg, CirclePatternCfg):
            self._update_circle(p_cfg, state)  # type: ignore[misc]
        elif isinstance(p_cfg, LemniscatePatternCfg):
            self._update_lemniscate(p_cfg, state)  # type: ignore[misc]
        elif isinstance(p_cfg, LissajousPatternCfg):
            self._update_lissajous(p_cfg, state)  # type: ignore[misc]
        elif isinstance(p_cfg, CapsulePatternCfg):
            self._update_capsule(p_cfg, state)  # type: ignore[misc]
        elif isinstance(p_cfg, SpiralPatternCfg):
            self._update_spiral(p_cfg, state)  # type: ignore[misc]
        return _get_transform_msg(state.current_pos, state.current_quat)

    def _update_line(self, cfg: LinePatternCfg, state: LinePatternState):
        state.dist_traveled += state.path_direction * self._abs_step_size

        if state.dist_traveled >= cfg.length:
            state.dist_traveled = cfg.length
            state.path_direction = -1
        elif state.dist_traveled <= 0:
            state.dist_traveled = 0
            state.path_direction = 1
            state.is_done = True  # A full back-and-forth trip is complete

        local_offset = np.array([state.dist_traveled * self._velocity_sign, 0, 0])
        local_yaw = 0 if state.path_direction > 0 else math.pi

        state.current_pos = cfg.initial_pos + self._initial_rot_mat @ local_offset
        state.current_quat = multiply_quats(
            cfg.initial_quat,
            yaw_to_quat(local_yaw + self._reverse_yaw_flip),
        )

    def _update_rectangle(self, cfg: RectanglePatternCfg, state: RectanglePatternState):
        w, h, dir_sign = (
            cfg.width,
            cfg.height,
            1 if cfg.direction == "counter-clockwise" else -1,
        )
        offsets = [
            np.array([0, 0]),
            np.array([w, 0]),
            np.array([w, h * dir_sign]),
            np.array([0, h * dir_sign]),
        ]
        dirs = [
            np.array([1, 0]),
            np.array([0, dir_sign]),
            np.array([-1, 0]),
            np.array([0, -dir_sign]),
        ]
        yaws = [0, dir_sign * math.pi / 2, math.pi, dir_sign * 3 * math.pi / 2]

        state.dist_on_segment += self._abs_step_size * self._velocity_sign
        # A loop is one perimeter in the direction of travel; reverse traversal
        # enters segment 3 on its first step, so segment indices cannot mark it.
        state.distance_traveled += self._abs_step_size
        perimeter = 2.0 * (w + h)
        if state.distance_traveled >= perimeter - 1e-9:
            state.distance_traveled -= perimeter
            state.is_done = True

        if state.segment_idx % 2 == 0:  # Horizontal segments
            segment_length = w
        else:  # Vertical segments
            segment_length = h

        if state.dist_on_segment >= segment_length:
            state.dist_on_segment -= segment_length
            state.segment_idx = (state.segment_idx + 1) % 4
        elif state.dist_on_segment < 0:
            state.segment_idx = (state.segment_idx - 1) % 4
            # Use the destination segment length to preserve overshoot when sides differ.
            state.dist_on_segment += w if state.segment_idx % 2 == 0 else h

        local_offset = (
            offsets[state.segment_idx] + dirs[state.segment_idx] * state.dist_on_segment
        )
        local_yaw = yaws[state.segment_idx]

        state.current_pos = cfg.initial_pos + self._initial_rot_mat @ np.array(
            [
                local_offset[0],
                local_offset[1],
                0,
            ]
        )
        state.current_quat = multiply_quats(
            cfg.initial_quat,
            yaw_to_quat(local_yaw + self._reverse_yaw_flip),
        )

    def _update_circle(self, cfg: CirclePatternCfg, state: CirclePatternState):
        state.angle += self._abs_step_size / (cfg.radius if cfg.radius > 0 else 1)
        if state.angle >= 2 * math.pi:
            state.angle = 0.0
            state.is_done = True

        a = state.angle * self._velocity_sign

        if cfg.direction == "counter-clockwise":
            local_offset = np.array(
                [
                    cfg.radius * math.sin(a),
                    cfg.radius * (1 - math.cos(a)),
                    0,
                ]
            )
            local_yaw = a
        else:  # Clockwise
            local_offset = np.array(
                [
                    cfg.radius * math.sin(a),
                    -cfg.radius * (1 - math.cos(a)),
                    0,
                ]
            )
            local_yaw = -a

        state.current_pos = cfg.initial_pos + self._initial_rot_mat @ local_offset
        state.current_quat = multiply_quats(cfg.initial_quat, yaw_to_quat(local_yaw))

    def _initialize_lemniscate_compensation(self):
        dir_sign = -1.0 if self.pattern_cfg.direction == "counter-clockwise" else 1.0
        # Rotate the initial tangent at t=pi/2 onto +X.
        self._inf_lemniscate_start_yaw = math.atan2(-0.5 * dir_sign, -0.5)
        angle = -self._inf_lemniscate_start_yaw
        c, s = math.cos(angle), math.sin(angle)
        self._inf_lemniscate_compensation_rot = np.array([[c, -s], [s, c]])

    def _update_lemniscate(
        self, cfg: LemniscatePatternCfg, state: LemniscatePatternState
    ):
        dir_sign = -1.0 if cfg.direction == "counter-clockwise" else 1.0

        def get_raw_lemniscate(t: float):
            cos_t, sin_t = math.cos(t), math.sin(t)
            den = 1 + sin_t**2
            if den == 0:
                return np.array([0, 0, 0])
            x = cfg.scale * cos_t / den
            y = cfg.scale * dir_sign * sin_t * cos_t / den
            return np.array([x, y, 0])

        t_start = math.pi / 2
        t_current = state.t + t_start

        eps = 1e-6
        pos_t1_raw = get_raw_lemniscate(t_current)
        pos_t2_raw = get_raw_lemniscate(t_current + eps * self._velocity_sign)
        tangent_raw = pos_t2_raw - pos_t1_raw

        pos_t1_comp = self._inf_lemniscate_compensation_rot @ pos_t1_raw[:2]
        pos_t2_comp = self._inf_lemniscate_compensation_rot @ pos_t2_raw[:2]
        inst_speed = np.linalg.norm(pos_t2_comp - pos_t1_comp) / eps

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

        state.current_pos = cfg.initial_pos + self._initial_rot_mat @ np.array(
            [
                compensated_offset_2d[0],
                compensated_offset_2d[1],
                0,
            ]
        )
        state.current_quat = multiply_quats(
            cfg.initial_quat,
            yaw_to_quat(compensated_yaw + self._reverse_yaw_flip),
        )

    def _initialize_lissajous_compensation(self):
        dir_sign = 1.0 if self.pattern_cfg.direction == "counter-clockwise" else -1.0
        # The initial tangent of (sin(2t), dir*sin(t)) is (2, dir).
        self._inf_intrinsic_start_yaw = math.atan2(dir_sign, 2.0)
        # Cancel its yaw with a 2D rotation.
        c = math.cos(-self._inf_intrinsic_start_yaw)
        s = math.sin(-self._inf_intrinsic_start_yaw)
        self._inf_compensation_rot = np.array([[c, -s], [s, c]])

    def _update_lissajous(self, cfg: LissajousPatternCfg, state: LissajousPatternState):
        dir_sign = 1.0 if cfg.direction == "counter-clockwise" else -1.0

        def get_local_pos_at_t(t):
            return np.array(
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
        inst_speed = np.linalg.norm(tangent_vec_uncompensated) / eps
        if inst_speed > 1e-5:
            state.t += self._abs_step_size * self._velocity_sign / inst_speed

        if abs(state.t) >= 2 * math.pi:
            state.t, state.is_done = 0.0, True

        local_offset_uncompensated = get_local_pos_at_t(state.t)

        compensated_offset_2d = (
            self._inf_compensation_rot @ local_offset_uncompensated[:2]
        )

        state.current_pos = cfg.initial_pos + self._initial_rot_mat @ np.array(
            [
                compensated_offset_2d[0],
                compensated_offset_2d[1],
                0,
            ]
        )

        tangent_yaw_uncompensated = math.atan2(
            tangent_vec_uncompensated[1], tangent_vec_uncompensated[0]
        )

        local_yaw = tangent_yaw_uncompensated - self._inf_intrinsic_start_yaw

        state.current_quat = multiply_quats(
            cfg.initial_quat, yaw_to_quat(local_yaw + self._reverse_yaw_flip)
        )

    def _update_capsule(self, cfg: CapsulePatternCfg, state: CapsulePatternState):
        length, radius, dir_sign = (
            cfg.length,
            cfg.radius,
            1 if cfg.direction == "counter-clockwise" else -1,
        )

        state.dist_on_segment += self._abs_step_size * self._velocity_sign
        state.distance_traveled += self._abs_step_size
        perimeter = sum(self._cap_segment_lengths)
        if state.distance_traveled >= perimeter - 1e-9:
            state.distance_traveled -= perimeter
            state.is_done = True

        if state.dist_on_segment >= self._cap_segment_lengths[state.segment_idx]:
            state.dist_on_segment -= self._cap_segment_lengths[state.segment_idx]
            state.segment_idx = (state.segment_idx + 1) % 4
        elif state.dist_on_segment < 0:
            state.segment_idx = (state.segment_idx - 1) % 4
            state.dist_on_segment += self._cap_segment_lengths[state.segment_idx]

        dist = state.dist_on_segment
        if state.segment_idx == 0:  # First straight line
            offset, yaw = np.array([dist, 0, 0]), 0
        elif state.segment_idx == 1:  # First arc
            phi = dist / radius
            center = np.array([length, dir_sign * radius, 0])
            angle = dir_sign * phi - (math.pi / 2 * dir_sign)
            offset = center + np.array(
                [
                    radius * math.cos(angle),
                    radius * math.sin(angle),
                    0,
                ]
            )
            yaw = dir_sign * phi
        elif state.segment_idx == 2:  # Second straight line
            offset, yaw = (
                np.array([length - dist, 2 * radius * dir_sign, 0]),
                math.pi,
            )
        else:  # state.segment_idx == 3, Second arc
            phi = dist / radius
            center = np.array([0, dir_sign * radius, 0])
            angle = dir_sign * phi + (math.pi / 2 * dir_sign)
            offset = center + np.array(
                [
                    radius * math.cos(angle),
                    radius * math.sin(angle),
                    0,
                ]
            )
            yaw = math.pi + dir_sign * phi

        state.current_pos = cfg.initial_pos + self._initial_rot_mat @ offset
        state.current_quat = multiply_quats(
            cfg.initial_quat,
            yaw_to_quat(yaw + self._reverse_yaw_flip),
        )

    def _update_spiral(self, cfg: SpiralPatternCfg, state: SpiralPatternState):
        p = state.phase
        N = cfg.n_loops
        R = cfg.max_radius
        dir_sign = 1 if cfg.direction == "counter-clockwise" else -1

        # Radius grows on [0, 1], shrinks on [1, 2], and angle stays continuous.
        C = dir_sign * 2 * math.pi * N  # Angular constant

        def get_derivatives_at_phase(phase_val):
            # Avoid a singular derivative at the center.
            phase_val = max(phase_val, 1e-6)

            if 0.0 <= phase_val < 1.0:
                r = R * phase_val
                dr_dp = R
            else:  # 1.0 <= phase_val <= 2.0
                r = R * (2.0 - phase_val)
                dr_dp = -R

            theta = C * phase_val
            dtheta_dp = C

            cos_t, sin_t = math.cos(theta), math.sin(theta)
            dx_dp = dr_dp * cos_t - r * dtheta_dp * sin_t
            dy_dp = dr_dp * sin_t + r * dtheta_dp * cos_t

            return dx_dp, dy_dp

        dx_dp, dy_dp = get_derivatives_at_phase(p)
        inst_speed = math.sqrt(dx_dp**2 + dy_dp**2)

        if inst_speed > 1e-5:
            state.phase += self._abs_step_size * self._velocity_sign / inst_speed

        if self._velocity_sign > 0 and state.phase >= 2.0:
            state.phase = 0.0
            state.is_done = True
        elif self._velocity_sign < 0 and state.phase <= 0.0:
            state.phase = 2.0
            state.is_done = True

        p_final = np.clip(state.phase, 0.0, 2.0)

        if 0.0 <= p_final < 1.0:
            r_final = R * p_final
        else:
            r_final = R * (2.0 - p_final)
        theta_final = C * p_final

        local_offset = np.array(
            [
                r_final * math.cos(theta_final),
                r_final * math.sin(theta_final),
                0,
            ]
        )

        dx_dp_final, dy_dp_final = get_derivatives_at_phase(p_final)
        local_yaw = math.atan2(dy_dp_final, dx_dp_final)

        state.current_pos = cfg.initial_pos + self._initial_rot_mat @ local_offset
        state.current_quat = multiply_quats(
            cfg.initial_quat,
            yaw_to_quat(local_yaw + self._reverse_yaw_flip),
        )

    def start(self):
        if self.is_running:
            return
        if self.cfg.n_loops == 0:
            # Without a timer, a running state would strand wait-loop callers.
            self._ros_node.get_logger().info(
                "Trajectory broadcaster not started (n_loops == 0)."
            )
            return
        self.is_running = True
        self._timer = self._ros_node.create_timer(1.0 / self.cfg.rate, self.cb)
        self._ros_runtime.start_spin()

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
        self.stop()
        self._ros_runtime.close()


def main():
    print("Initializing trajectory broadcaster...")

    pattern_cfg = CapsulePatternCfg(
        length=3.0,
        radius=1.15,
        direction="counter-clockwise",
        initial_pos=np.array([2.1, -3.9, 0.0], dtype=np.float32),
        initial_quat=yaw_to_quat(0.0, deg=True),
    )
    # Set n_loops to -1 for infinite looping.
    # Set velocity to a negative value to run the trajectory backward.
    traj_cfg = RosTfTrajectoryGeneratorCfg(
        pattern=pattern_cfg,
        n_loops=-1,
        velocity=0.25,
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
            "\nError: Could not import ROS 2 libraries. Install ROS 2 and source its setup file."
        )
    finally:
        if broadcaster:
            print("Shutting down...")
            broadcaster.shutdown()
            print("Shutdown complete.")


if __name__ == "__main__":
    main()
