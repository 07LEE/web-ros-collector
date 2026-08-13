"""Camera bridge module for handling compressed image frame publishing in ROS2."""

import json

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class CameraBridge:
    """Handles compressed camera image topic publishing without backend decoding overhead."""

    def __init__(self, node, topic_name: str = 'image_raw/compressed', frame_id: str = 'phone_camera'):
        """Initializes publisher for CompressedImage and exposure metadata topics.

        Args:
            node (rclpy.node.Node): Parent ROS2 node instance.
            topic_name (str): Topic name to publish compressed images.
            frame_id (str): Frame ID for header.
        """
        self.node = node
        self.frame_id = frame_id
        self.compressed_publisher = self.node.create_publisher(
            CompressedImage,
            topic_name,
            qos_profile_sensor_data
        )
        self.exposure_publisher = self.node.create_publisher(
            String,
            '/camera/exposure_metadata',
            qos_profile_sensor_data
        )

    def handle_upload(self, post_data: bytes, content_type: str, stamp,
                      exposure_time: str = None, iso: str = None) -> None:
        """Processes compressed image payload received over HTTP and publishes to ROS2 topics.

        Publishes the raw compressed image to the image topic and, if exposure
        metadata is present, a companion JSON message to /camera/exposure_metadata
        with the same timestamp for post-processing and frame selection.

        Args:
            post_data (bytes): Raw compressed binary image payload (JPEG/PNG).
            content_type (str): HTTP Content-Type header.
            stamp (builtin_interfaces.msg.Time): ROS2 time stamp.
            exposure_time (str): Exposure time value in 100us units (W3C spec), or None.
            iso (str): ISO sensitivity value, or None.
        """
        img_format = 'png' if 'png' in content_type else 'jpeg'
        comp_msg = CompressedImage()
        comp_msg.header.stamp = stamp
        comp_msg.header.frame_id = self.frame_id
        comp_msg.format = img_format
        comp_msg.data = post_data
        self.compressed_publisher.publish(comp_msg)

        if exposure_time is not None or iso is not None:
            meta = {
                'stamp_sec': stamp.sec,
                'stamp_nanosec': stamp.nanosec,
                'exposure_time_100us': int(exposure_time) if exposure_time else None,
                'exposure_time_us': int(exposure_time) * 100 if exposure_time else None,
                'iso': int(iso) if iso else None,
            }
            meta_msg = String()
            meta_msg.data = json.dumps(meta)
            self.exposure_publisher.publish(meta_msg)
