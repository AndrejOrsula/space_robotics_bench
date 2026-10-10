from __future__ import annotations

import time
from collections.abc import Callable
from functools import partial
from typing import TYPE_CHECKING, Any, Sequence

import gymnasium
import numpy as np
from pydantic import Field

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
)
from srb.utils import logging
from srb.utils.failures import (
    FailureCategory,
    FailureInfo,
    HealthEvent,
    HealthSeverity,
    HealthStatus,
)

cv2: Any = None
try:
    import cv2 as _opencv
except ImportError:  # pragma: no cover - optional hardware image dependency
    pass
else:
    cv2 = _opencv

if TYPE_CHECKING:
    from rclpy.subscription import Subscription
    from sensor_msgs.msg import Image


_KNOWN_IMAGE_BYTES_PER_PIXEL = {
    "rgb8": 3,
    "bgr8": 3,
    "mono8": 1,
    "8uc1": 1,
    "16uc1": 2,
    "mono16": 2,
    "32fc1": 4,
    "32fc": 4,
}


class RosImageInterfaceCfg(HardwareInterfaceCfg):
    """Configuration for the ROS image interface."""

    topics: dict[str, str] = Field(default_factory=dict)
    """A dictionary mapping camera names to ROS topic names."""
    output_width: int = Field(default=128, gt=0)
    """The width of the output image."""
    output_height: int = Field(default=128, gt=0)
    """The height of the output image."""
    observation_name: str = "pixels"
    """The name of the observation in the output dictionary."""
    max_frame_age_s: float = Field(default=0.5, gt=0.0, allow_inf_nan=False)
    """Maximum monotonic age of every required decoded camera frame."""


class RosImageInterface(HardwareInterface):
    """Receive RGB and depth ROS images as a concatenated, resized observation.

    Camera names containing ``depth`` select depth handling; other cameras are
    treated as color images.
    """

    cfg: RosImageInterfaceCfg
    CUSTOM_ALIASES: Sequence[Sequence[str]] = (("pixels", "image", "img", "rgbd"),)

    def __init__(
        self,
        cfg: RosImageInterfaceCfg | None = None,
        *,
        clock: Callable[[], float] = time.monotonic,
    ):
        """Initialize the ROS image interface."""
        if cfg is None:
            cfg = RosImageInterfaceCfg()
        super().__init__(cfg)
        self._clock = clock
        self.image_data: dict[str, np.ndarray] = {}
        self._last_frame_s: dict[str, float] = {}
        self.subscribers: list[Subscription] = []
        # Keep one failure per source and expose the newest without an event log.
        self._last_failures: dict[str, FailureInfo] = {}
        self._last_failure: FailureInfo | None = None
        self._last_failure_source: str | None = None
        self._decoder_fallback_failure: FailureInfo | None = None
        self._processing_fallback_failure: FailureInfo | None = None

    def start(self, **kwargs):
        """Create subscribers for configured topics."""
        super().start(**kwargs)
        if not self.cfg.topics:
            logging.warning(
                f"[{self.name}] No topics configured. The interface will do nothing."
            )
            return

        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import Image

        for name, topic in self.cfg.topics.items():
            callback = partial(self._image_callback, camera_name=name)
            sub = self.ros_node.create_subscription(
                Image,
                topic,
                callback,
                QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                ),
            )
            self.subscribers.append(sub)
            logging.info(f"[{self.name}] Subscribing to '{topic}' for camera '{name}'")

    def close(self):
        """Destroy all subscribers and close the interface."""
        super().close()
        for sub in self.subscribers:
            sub.destroy()
        self.subscribers.clear()

    def reset(self):
        """Clear received frames and failure state."""
        super().reset()
        self.image_data.clear()
        self._last_frame_s.clear()
        self._last_failures.clear()
        self._last_failure = None
        self._last_failure_source = None
        self._decoder_fallback_failure = None
        self._processing_fallback_failure = None

    def _image_callback(self, msg: "Image", camera_name: str):
        """Validate, decode, resize, and store an incoming ROS image."""
        received_s = self._clock()
        if camera_name not in self.cfg.topics:
            cause = ValueError(f"camera {camera_name!r} is not configured")
            self._record_failure(
                camera_name,
                self._image_failure(
                    category=FailureCategory.INVALID_INPUT,
                    code="ros.image.unknown_camera",
                    message=str(cause),
                    stage="routing",
                    msg=msg,
                    camera_name=camera_name,
                    cause=cause,
                ),
            )
            logging.error(
                "[%s] rejected image for unknown camera %s", self.name, camera_name
            )
            return
        is_depth = "depth" in camera_name.lower()
        # Convert color to BGR and preserve depth encoding for metre scaling;
        # requesting 32FC1 would cast millimetres without scaling.
        desired_encoding = "passthrough" if is_depth else "bgr8"

        try:
            self._validate_ros_image_message(msg)
        except Exception as exc:  # noqa: BLE001 - ROS callback boundary
            self._record_failure(
                camera_name,
                self._image_failure(
                    category=FailureCategory.INVALID_INPUT,
                    code="ros.image.frame_malformed",
                    message=f"ROS image message rejected: {exc}",
                    stage="message",
                    msg=msg,
                    camera_name=camera_name,
                    cause=exc,
                ),
            )
            logging.error(
                "[%s] rejected malformed image message for %s: %s",
                self.name,
                camera_name,
                exc,
            )
            return

        try:
            from cv_bridge import CvBridge  # type: ignore[import-untyped]
        except ImportError:
            self._decoder_fallback_failure = FailureInfo(
                category=FailureCategory.OPTIONAL_DEPENDENCY_UNAVAILABLE,
                code="ros.image.cv_bridge_unavailable",
                message="cv_bridge is unavailable; using the local NumPy decoder",
                component=self.name,
                details={
                    "camera": camera_name,
                    "topic": self.cfg.topics.get(camera_name),
                    "primary_decoder": "cv_bridge",
                    "substitute_decoder": "numpy",
                },
                retryable=False,
                fallback_allowed=True,
                evidence_valid=False,
            )
            logging.warning(
                "cv_bridge not found, using a slower numpy-based conversion."
            )
            try:
                cv_image = self._numpy_imgmsg_to_cv2(msg, is_depth)
            except Exception as exc:  # noqa: BLE001 - ROS callback boundary
                self._record_failure(
                    camera_name,
                    self._image_failure(
                        category=self._image_exception_category(
                            exc, default=FailureCategory.INVALID_INPUT
                        ),
                        code="ros.image.decode_failed",
                        message=f"NumPy image decode failed: {exc}",
                        stage="numpy",
                        msg=msg,
                        camera_name=camera_name,
                        cause=exc,
                    ),
                )
                logging.error(
                    "[%s] image decode failed for %s: %s",
                    self.name,
                    camera_name,
                    exc,
                )
                return
        else:
            try:
                bridge = CvBridge()
                # The bridge will handle conversion from different encodings.
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding=desired_encoding)
                if is_depth:
                    cv_image = self._depth_to_metres(cv_image, msg.encoding)
            except Exception as exc:  # noqa: BLE001 - ROS callback boundary
                self._record_failure(
                    camera_name,
                    self._image_failure(
                        category=self._image_exception_category(
                            exc, default=FailureCategory.INTEGRITY_FAILURE
                        ),
                        code="ros.image.decode_failed",
                        message=f"cv_bridge image decode failed: {exc}",
                        stage="cv_bridge",
                        msg=msg,
                        camera_name=camera_name,
                        cause=exc,
                    ),
                )
                logging.error(
                    "[%s] image decode failed for %s: %s",
                    self.name,
                    camera_name,
                    exc,
                )
                return

        try:
            cv_image = self._validate_decoded_image(cv_image, is_depth=is_depth)
            resized_image = self._resize_image(
                cv_image,
                (self.cfg.output_width, self.cfg.output_height),
                nearest=is_depth,
            )

            if is_depth:
                if resized_image.ndim == 2:
                    resized_image = np.expand_dims(resized_image, axis=-1)
            elif resized_image.ndim == 3 and resized_image.shape[2] == 3:
                # Convert BGR to RGB.
                resized_image = self._convert_color(resized_image, rgb_to_bgr=False)
            resized_image = self._validate_decoded_image(
                resized_image, is_depth=is_depth
            )
        except Exception as exc:  # noqa: BLE001 - ROS callback boundary
            self._record_failure(
                camera_name,
                self._image_failure(
                    category=(
                        FailureCategory.INTEGRITY_FAILURE
                        if "finite" in str(exc).lower()
                        else self._image_exception_category(
                            exc, default=FailureCategory.INVALID_INPUT
                        )
                    ),
                    code="ros.image.frame_invalid",
                    message=f"decoded image frame rejected: {exc}",
                    stage="validation",
                    msg=msg,
                    camera_name=camera_name,
                    cause=exc,
                ),
            )
            logging.error(
                "[%s] rejected image frame for %s: %s",
                self.name,
                camera_name,
                exc,
            )
            return

        self.image_data[camera_name] = resized_image
        self._last_frame_s[camera_name] = received_s
        self._clear_failure(camera_name)

    @staticmethod
    def _image_exception_category(
        exc: BaseException, *, default: FailureCategory
    ) -> FailureCategory:
        if isinstance(exc, OverflowError):
            return FailureCategory.INTEGRITY_FAILURE
        if isinstance(exc, (TypeError, ValueError)):
            return default
        return FailureCategory.INTERNAL_INVARIANT

    def _image_failure(
        self,
        *,
        category: FailureCategory,
        code: str,
        message: str,
        stage: str,
        msg: object,
        camera_name: str,
        cause: BaseException,
    ) -> FailureInfo:
        """Build bounded, JSON-shaped evidence for one rejected frame."""
        encoding = getattr(msg, "encoding", None)
        if not isinstance(encoding, str):
            encoding = type(encoding).__name__
        return FailureInfo(
            category=category,
            code=code,
            message=message,
            component=self.name,
            details={
                "camera": camera_name,
                "topic": self.cfg.topics.get(camera_name),
                "stage": stage,
                "encoding": encoding,
                "cause_type": f"{type(cause).__module__}.{type(cause).__qualname__}",
                "cause_message": str(cause) or type(cause).__name__,
            },
            retryable=False,
            fallback_allowed=False,
            evidence_valid=False,
        )

    def _record_failure(self, camera_name: str, failure: FailureInfo) -> None:
        key = camera_name if camera_name in self.cfg.topics else "<unknown>"
        self._last_failures.pop(key, None)
        self._last_failures[key] = failure
        self._last_failure_source = key
        self._last_failure = failure

    def _clear_failure(self, camera_name: str) -> None:
        key = camera_name if camera_name in self.cfg.topics else "<unknown>"
        self._last_failures.pop(key, None)
        if self._last_failures:
            source = next(reversed(self._last_failures))
            self._last_failure_source = source
            self._last_failure = self._last_failures[source]
        else:
            self._last_failure_source = None
            self._last_failure = None

    @staticmethod
    def _validate_decoded_image(data: np.ndarray, *, is_depth: bool) -> np.ndarray:
        """Reject malformed shape, dtype, and non-finite decoded frames."""
        if not isinstance(data, np.ndarray):
            raise TypeError("decoder returned a non-NumPy image")
        if data.size == 0:
            raise ValueError("decoded image is empty")
        if data.dtype.kind not in "iuf":
            raise TypeError(f"decoded image dtype {data.dtype} is not numeric")
        if not np.all(np.isfinite(data)):
            raise ValueError("decoded image contains non-finite values")
        if is_depth:
            if np.any(data < 0):
                raise ValueError("depth image contains negative values")
            if data.ndim == 3 and data.shape[-1] == 1:
                return data
            if data.ndim != 2:
                raise ValueError(
                    f"depth image must be 2D or single-channel 3D, got {data.shape}"
                )
            return data
        if data.ndim != 3 or data.shape[-1] != 3:
            raise ValueError(
                f"color image must have shape (height, width, 3), got {data.shape}"
            )
        return data

    @classmethod
    def _validate_ros_image_message(cls, msg: object) -> None:
        """Check message framing before either decoder is allowed to run."""
        encoding = getattr(msg, "encoding", None)
        if not isinstance(encoding, str) or not encoding.strip():
            raise TypeError("image encoding must be a non-empty string")
        height = cls._positive_image_dimension(getattr(msg, "height", None), "height")
        width = cls._positive_image_dimension(getattr(msg, "width", None), "width")
        step = cls._positive_image_dimension(getattr(msg, "step", None), "step")
        bytes_per_pixel = _KNOWN_IMAGE_BYTES_PER_PIXEL.get(encoding.lower())
        if bytes_per_pixel is not None:
            minimum_row_bytes = width * bytes_per_pixel
            if step < minimum_row_bytes:
                raise ValueError(
                    f"image step {step} is smaller than the minimum row size "
                    f"{minimum_row_bytes} for {encoding}"
                )
        try:
            buffer = memoryview(getattr(msg, "data", None)).cast("B")
        except (TypeError, ValueError) as exc:
            raise TypeError("image data must expose a byte buffer") from exc
        required_bytes = height * step
        if buffer.nbytes < required_bytes:
            raise ValueError(
                f"image data has {buffer.nbytes} bytes, needs {required_bytes}"
            )

    @staticmethod
    def _depth_to_metres(data: np.ndarray, encoding: str) -> np.ndarray:
        """Return depth in metres; uint16/16UC1 values are millimetres.

        Float input is assumed to be metres. Detect by dtype or encoding so
        nonstandard encoding casing still scales.
        """
        if data.dtype == np.uint16 or (encoding or "").lower() == "16uc1":
            return data.astype(np.float32) / 1000.0
        return data.astype(np.float32)

    def _resize_image(
        self,
        data: np.ndarray,
        size: tuple[int, int],
        *,
        nearest: bool,
    ) -> np.ndarray:
        """Resize an image, retaining a NumPy path when OpenCV is absent."""
        if cv2 is not None:
            return cv2.resize(
                data,
                size,
                interpolation=cv2.INTER_NEAREST if nearest else cv2.INTER_AREA,
            )

        self._record_opencv_fallback()
        target_width, target_height = size
        source_height, source_width = data.shape[:2]
        row_indices = np.minimum(
            np.arange(target_height) * source_height // target_height,
            source_height - 1,
        )
        column_indices = np.minimum(
            np.arange(target_width) * source_width // target_width,
            source_width - 1,
        )
        return data[np.ix_(row_indices, column_indices)]

    def _convert_color(self, data: np.ndarray, *, rgb_to_bgr: bool) -> np.ndarray:
        """Swap RGB/BGR channels with OpenCV or a NumPy fallback."""
        if cv2 is not None:
            code = cv2.COLOR_RGB2BGR if rgb_to_bgr else cv2.COLOR_BGR2RGB
            return cv2.cvtColor(data, code)
        self._record_opencv_fallback()
        if data.ndim == 3 and data.shape[-1] == 3:
            return data[..., ::-1]
        return data

    def _record_opencv_fallback(self) -> None:
        """Retain one bounded health fact when NumPy substitutes for OpenCV."""
        if self._processing_fallback_failure is not None:
            return
        self._processing_fallback_failure = FailureInfo(
            category=FailureCategory.OPTIONAL_DEPENDENCY_UNAVAILABLE,
            code="ros.image.opencv_unavailable",
            message=(
                "OpenCV is unavailable; using NumPy image resize/color processing"
            ),
            component=self.name,
            details={
                "primary_processor": "opencv",
                "substitute_processor": "numpy",
            },
            retryable=False,
            fallback_allowed=True,
            evidence_valid=False,
        )

    def _numpy_imgmsg_to_cv2(self, msg: "Image", is_depth: bool) -> np.ndarray:
        """Convert a ROS image without silently accepting malformed bytes."""
        encoding = getattr(msg, "encoding", None)
        if not isinstance(encoding, str) or not encoding.strip():
            raise TypeError("image encoding must be a non-empty string")
        encoding_lower = encoding.lower()

        if encoding_lower in {"rgb8", "bgr8"}:
            dtype = np.dtype(np.uint8)
            channels = 3
        elif encoding_lower in {"16uc1", "mono16"}:
            dtype = np.dtype(np.uint16)
            channels = 1
        elif encoding_lower in {"32fc1", "32fc"}:
            dtype = np.dtype(np.float32)
            channels = 1
        elif is_depth and encoding_lower in {"8uc1", "mono8"}:
            dtype = np.dtype(np.uint8)
            channels = 1
        else:
            raise ValueError(f"unsupported image encoding: {encoding}")

        height = self._positive_image_dimension(getattr(msg, "height", None), "height")
        width = self._positive_image_dimension(getattr(msg, "width", None), "width")
        step = self._positive_image_dimension(getattr(msg, "step", None), "step")
        dtype = dtype.newbyteorder(">" if getattr(msg, "is_bigendian", False) else "<")
        row_bytes = width * channels * dtype.itemsize
        if step < row_bytes:
            raise ValueError(
                f"image step {step} is smaller than one packed row ({row_bytes})"
            )
        if step % dtype.itemsize:
            raise ValueError("image step is not aligned to the encoding dtype")

        raw = getattr(msg, "data", None)
        try:
            buffer = memoryview(raw).cast("B")
        except (TypeError, ValueError) as exc:
            raise TypeError("image data must expose a byte buffer") from exc
        required_bytes = height * step
        if buffer.nbytes < required_bytes:
            raise ValueError(
                f"image data has {buffer.nbytes} bytes, needs {required_bytes}"
            )

        row_values = np.frombuffer(
            buffer,
            dtype=dtype,
            count=height * (step // dtype.itemsize),
        ).reshape(height, step // dtype.itemsize)
        packed = row_values[:, : width * channels]
        data = (
            packed.reshape(height, width, channels)
            if channels > 1
            else packed.reshape(height, width)
        )
        # Consumers (torch.from_numpy included) need native byte order.
        data = data.astype(dtype.newbyteorder("="), copy=False)

        if is_depth:
            return self._depth_to_metres(data, encoding)
        if encoding_lower == "rgb8":
            return self._convert_color(data, rgb_to_bgr=True)
        return data

    @staticmethod
    def _positive_image_dimension(value: object, name: str) -> int:
        if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
            raise TypeError(f"image {name} must be an integer")
        result = int(value)
        if result <= 0:
            raise ValueError(f"image {name} must be positive")
        return result

    @property
    def observation(self) -> dict[str, np.ndarray]:
        """Return resized images concatenated by camera name, or empty if incomplete."""
        if not self.cfg.topics:
            return {}

        ordered_camera_names = sorted(self.cfg.topics.keys())
        # Wait until every configured camera has produced a frame.
        if any(name not in self.image_data for name in ordered_camera_names):
            return {}

        ordered_images = [self.image_data[name] for name in ordered_camera_names]

        # Concatenation uses float if any input (such as depth) is float.
        combined_image = np.concatenate(ordered_images, axis=-1)

        return {self.cfg.observation_name: combined_image}

    def health(self) -> HealthEvent:
        """Report readiness and latest decoder evidence.

        Only a valid frame from the failed camera clears its failure. NumPy
        fallback remains degraded until reset while the primary decoder is absent.
        """
        required_topics = tuple(sorted(self.cfg.topics))
        received_topics = tuple(
            name for name in required_topics if name in self.image_data
        )
        missing_topics = tuple(
            name for name in required_topics if name not in self.image_data
        )
        now = self._clock()
        frame_ages_s = {
            name: max(0.0, now - self._last_frame_s[name])
            for name in received_topics
            if name in self._last_frame_s
        }
        stale_topics = tuple(
            name
            for name in received_topics
            if name not in frame_ages_s or frame_ages_s[name] > self.cfg.max_frame_age_s
        )
        failure = self._last_failure
        if failure is not None:
            state = "failure"
            status = HealthStatus.UNHEALTHY
            severity = HealthSeverity.ERROR
            code = failure.code
            message = f"latest image frame was rejected: {failure.message}"
        elif not required_topics:
            state = "no_topics"
            status = HealthStatus.UNHEALTHY
            severity = HealthSeverity.ERROR
            code = f"{self.name}.{state}"
            message = "no image topics are configured"
        elif not received_topics:
            state = "pre_message"
            status = HealthStatus.UNHEALTHY
            severity = HealthSeverity.ERROR
            code = f"{self.name}.{state}"
            message = "waiting for the first image message"
        elif missing_topics:
            state = "partial"
            status = HealthStatus.UNHEALTHY
            severity = HealthSeverity.ERROR
            code = f"{self.name}.{state}"
            message = "waiting for all configured image messages"
        elif stale_topics:
            state = "stale"
            status = HealthStatus.UNHEALTHY
            severity = HealthSeverity.ERROR
            code = f"{self.name}.{state}"
            message = "one or more required image frames are stale"
        elif (
            self._decoder_fallback_failure is not None
            or self._processing_fallback_failure is not None
        ):
            state = "degraded_fallback"
            status = HealthStatus.DEGRADED
            severity = HealthSeverity.WARNING
            if self._decoder_fallback_failure is not None:
                code = "ros.image.numpy_fallback"
                message = (
                    "all configured image topics have data, but cv_bridge is "
                    "unavailable and the NumPy decoder is active"
                )
            else:
                code = "ros.image.opencv_unavailable"
                message = (
                    "all configured image topics have data, but OpenCV is "
                    "unavailable and NumPy image processing is active"
                )
        else:
            state = "healthy"
            status = HealthStatus.HEALTHY
            severity = HealthSeverity.INFO
            code = f"{self.name}.{state}"
            message = "all configured image topics have produced a frame"
        return HealthEvent(
            status=status,
            severity=severity,
            component=self.name,
            code=code,
            message=message,
            details={
                "state": state,
                "required_topics": required_topics,
                "received_topics": received_topics,
                "missing_topics": missing_topics,
                "stale_topics": stale_topics,
                "frame_ages_ms": {
                    name: age_s * 1000.0 for name, age_s in frame_ages_s.items()
                },
                "max_frame_age_ms": self.cfg.max_frame_age_s * 1000.0,
                "required_messages": len(required_topics),
                "received_messages": len(received_topics),
                "ready": status is HealthStatus.HEALTHY,
                "last_failure_source": self._last_failure_source,
                "decoder_fallback": (
                    None
                    if self._decoder_fallback_failure is None
                    else self._decoder_fallback_failure.to_mapping()
                ),
                "processing_fallback": (
                    None
                    if self._processing_fallback_failure is None
                    else self._processing_fallback_failure.to_mapping()
                ),
            },
            failure=(
                failure
                if failure is not None
                else self._decoder_fallback_failure or self._processing_fallback_failure
                if status is HealthStatus.DEGRADED
                else None
            ),
        )

    @property
    def supported_observation_spaces(self) -> gymnasium.spaces.Dict:
        """Defines the observation space for the combined image."""
        if not self.cfg.topics:
            return gymnasium.spaces.Dict()

        ordered_camera_names = sorted(self.cfg.topics.keys())
        num_channels = 0
        has_depth = False

        for name in ordered_camera_names:
            if "depth" in name.lower():
                num_channels += 1
                has_depth = True
            else:
                num_channels += 3

        # If depth is present, all data will be cast to float32 on concatenation
        dtype = np.float32 if has_depth else np.uint8

        # Define bounds based on data type
        low = 0.0 if has_depth else 0
        high = np.inf if has_depth else 255

        return gymnasium.spaces.Dict(
            {
                self.cfg.observation_name: gymnasium.spaces.Box(
                    low=low,
                    high=high,
                    shape=(
                        self.cfg.output_height,
                        self.cfg.output_width,
                        num_channels,
                    ),
                    dtype=dtype,
                )
            }
        )
