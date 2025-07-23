from __future__ import annotations

from functools import partial
from typing import TYPE_CHECKING, Dict, List, Sequence

import cv2
import gymnasium
import numpy

from srb.interfaces.sim_to_real.core.hardware import (
    HardwareInterface,
    HardwareInterfaceCfg,
)
from srb.utils import logging

if TYPE_CHECKING:
    from rclpy.subscription import Subscription
    from sensor_msgs.msg import Image


class RosImageCfg(HardwareInterfaceCfg):
    """Configuration for the ROS image interface."""

    topics: Dict[str, str] = {}
    """A dictionary mapping camera names to ROS topic names."""
    output_width: int = 128
    """The width of the output image."""
    output_height: int = 128
    """The height of the output image."""
    observation_name: str = "pixels"
    """The name of the observation in the output dictionary."""


class RosImage(HardwareInterface):
    """
    A hardware interface for receiving and processing image data from ROS topics.

    This interface subscribes to one or more ROS topics publishing `sensor_msgs/Image`
    messages. It can handle both RGB and depth images, identified by "rgb" or "depth"
    in their camera names. The received images are resized to a common output
    dimension and then concatenated into a single multi-channel numpy array.
    """

    cfg: RosImageCfg
    CUSTOM_ALIASES: Sequence[Sequence[str]] = (("pixels", "image", "img", "rgbd"),)

    def __init__(self, cfg: RosImageCfg = RosImageCfg()):
        """Initializes the ROS image interface."""
        super().__init__(cfg)
        self.image_data: Dict[str, numpy.ndarray] = {}
        self.subscribers: List[Subscription] = []

    def start(self, **kwargs):
        """Starts the ROS image interface, creating subscribers for each topic."""
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
        """Closes the ROS image interface, destroying all subscribers."""
        super().close()
        for sub in self.subscribers:
            sub.destroy()
        self.subscribers.clear()

    def reset(self):
        """Resets the interface, clearing any received image data."""
        super().reset()
        self.image_data.clear()

    def _image_callback(self, msg: "Image", camera_name: str):
        """
        Callback function to process incoming ROS Image messages.
        Decodes, resizes, and stores the image data.
        """
        is_depth = "depth" in camera_name.lower()
        # Use 32FC1 for depth to handle float conversion, bgr8 for color
        desired_encoding = "32FC1" if is_depth else "bgr8"

        try:
            from cv_bridge import CvBridge  # type: ignore

            bridge = CvBridge()
            # The bridge will handle conversion from different encodings
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding=desired_encoding)
        except ImportError:
            logging.warning(
                "cv_bridge not found, using a slower numpy-based conversion."
            )
            cv_image = self._numpy_imgmsg_to_cv2(msg, is_depth)
            if cv_image is None:
                return

        resized_image = cv2.resize(
            cv_image,
            (self.cfg.output_width, self.cfg.output_height),
            interpolation=cv2.INTER_NEAREST if is_depth else cv2.INTER_AREA,
        )

        if is_depth:
            if resized_image.ndim == 2:
                resized_image = numpy.expand_dims(resized_image, axis=-1)
        elif resized_image.ndim == 3 and resized_image.shape[2] == 3:
            # Convert BGR to RGB
            resized_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)

        self.image_data[camera_name] = resized_image

    def _numpy_imgmsg_to_cv2(
        self, msg: "Image", is_depth: bool
    ) -> numpy.ndarray | None:
        """Converts a sensor_msgs/Image to a numpy array without cv_bridge."""
        if not msg.data:
            logging.warning("Received an image message with no data.")
            return None
        # Determine numpy dtype from encoding
        try:
            if "FC" in msg.encoding:
                dtype = numpy.float32
            elif "16" in msg.encoding:
                dtype = numpy.uint16
            else:
                dtype = numpy.uint8

            itemsize = numpy.dtype(dtype).itemsize
            channels = msg.step // (msg.width * itemsize)
            shape = (
                (msg.height, msg.width, channels)
                if channels > 1
                else (msg.height, msg.width)
            )
            data = numpy.frombuffer(msg.data, dtype=dtype).reshape(shape)
        except (ValueError, TypeError) as e:
            logging.error(f"Error converting image buffer: {e}")
            return None

        # Handle color format and depth unit conversions
        if is_depth:
            if msg.encoding == "16UC1":
                # Convert from mm to meters
                return data.astype(numpy.float32) / 1000.0
            elif msg.encoding == "32FC1":
                return data
        else:
            if msg.encoding == "rgb8":
                return cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "bgr8":
                return data

        logging.error(f"Unsupported image encoding in numpy fallback: {msg.encoding}")
        return None

    @property
    def observation(self) -> Dict[str, numpy.ndarray]:
        """
        Constructs and returns the combined image observation.
        Images are ordered by camera name, resized, and concatenated.
        Returns an empty dictionary if not all images have been received.
        """
        if not self.cfg.topics:
            return {}

        ordered_camera_names = sorted(self.cfg.topics.keys())
        # Return empty dict if we haven't received an image from all topics yet
        if len(self.image_data) < len(ordered_camera_names):
            return {}

        ordered_images = [self.image_data[name] for name in ordered_camera_names]

        # Concatenate images along the channel axis.
        # This will up-cast uint8 to float if float images (like depth) are present.
        combined_image = numpy.concatenate(ordered_images, axis=-1)

        return {self.cfg.observation_name: combined_image}

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
        dtype = numpy.float32 if has_depth else numpy.uint8

        # Define bounds based on data type
        low = 0.0 if has_depth else 0
        high = numpy.inf if has_depth else 255

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
