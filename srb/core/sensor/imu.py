# Patch Isaac Lab's concrete IMU backends so reset rows carry a reading.
#
# The backend skips environments whose sensor timestamp is still zero after a
# reset (solver accelerations are stale until the next physics step) and
# leaves zeros, so the first observation of every episode reported 0 m/s^2
# instead of the gravity reaction. Those rows now report the gyroscope from the
# body velocity and the accelerometer without the stale solver acceleration:
# gravity reaction plus the centripetal term of the sensor offset.

import functools
import importlib

import warp as wp


@wp.kernel
def _imu_reset_reading_kernel(
    env_mask: wp.array(dtype=wp.bool),
    transforms: wp.array(dtype=wp.transformf),
    velocities: wp.array(dtype=wp.spatial_vectorf),
    coms: wp.array(dtype=wp.transformf),
    offset_pos_b: wp.array(dtype=wp.vec3f),
    offset_quat_b: wp.array(dtype=wp.quatf),
    gravity_bias_w: wp.vec3f,
    timestamp: wp.array(dtype=wp.float32),
    out_ang_vel_b: wp.array(dtype=wp.vec3f),
    out_lin_acc_b: wp.array(dtype=wp.vec3f),
):
    idx = wp.tid()
    if not env_mask[idx] or timestamp[idx] != 0.0:
        return

    body_quat = wp.transform_get_rotation(transforms[idx])
    ang_vel_w = wp.spatial_bottom(velocities[idx])
    com_pos_b = wp.transform_get_translation(coms[idx])
    lever_arm = wp.quat_rotate(body_quat, offset_pos_b[idx] - com_pos_b)
    lin_acc_w = wp.cross(ang_vel_w, wp.cross(ang_vel_w, lever_arm)) + gravity_bias_w

    sensor_quat = body_quat * offset_quat_b[idx]
    out_ang_vel_b[idx] = wp.quat_rotate_inv(sensor_quat, ang_vel_w)
    out_lin_acc_b[idx] = wp.quat_rotate_inv(sensor_quat, lin_acc_w)


def _wrap_update(original):
    @functools.wraps(original)
    def _update_buffers_impl(self, env_mask=None):
        original(self, env_mask)
        wp.launch(
            _imu_reset_reading_kernel,
            dim=self._num_envs,
            inputs=[
                self._resolve_indices_and_mask(None, env_mask),
                self._raw_transforms,
                self._raw_velocities,
                self._coms_buffer,
                self._offset_pos_b,
                self._offset_quat_b,
                self._gravity_bias_w,
                self._timestamp,
                self._data._ang_vel_b,
                self._data._lin_acc_b,
            ],
            device=self._device,
        )

    _update_buffers_impl._srb_reset_reading = True
    return _update_buffers_impl


def _patch_imu_backend() -> None:
    try:
        module = importlib.import_module("isaaclab_physx.sensors.imu.imu")
    except ImportError:
        return
    original = module.Imu._update_buffers_impl
    if not getattr(original, "_srb_reset_reading", False):
        module.Imu._update_buffers_impl = _wrap_update(original)


_patch_imu_backend()
