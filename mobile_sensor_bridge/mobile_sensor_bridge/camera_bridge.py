"""Camera bridge module for handling compressed image frame publishing in ROS2."""

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class CameraBridge:
    """Handles compressed camera image topic publishing without backend decoding overhead."""

    def __init__(self, node, topic_name: str = 'image_raw/compressed', frame_id: str = 'phone_camera'):
        """Initializes publisher for CompressedImage topic.

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

    def handle_upload(self, post_data: bytes, content_type: str, stamp) -> None:
        """Processes compressed image payload received over HTTP and publishes to ROS2 topic.

        Args:
            post_data (bytes): Raw compressed binary image payload (JPEG/PNG).
            content_type (str): HTTP Content-Type header.
            stamp (builtin_interfaces.msg.Time): ROS2 time stamp.
        """
        img_format = 'png' if 'png' in content_type else 'jpeg'
        comp_msg = CompressedImage()
        comp_msg.header.stamp = stamp
        comp_msg.header.frame_id = self.frame_id
        comp_msg.format = img_format
        comp_msg.data = post_data

        self.compressed_publisher.publish(comp_msg)
