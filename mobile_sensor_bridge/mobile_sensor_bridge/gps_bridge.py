"""GPS Sensor Bridge module for ROS 2."""

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from rclpy.qos import qos_profile_sensor_data


class GpsBridge:
    """Handles parsing GPS Geolocation data from web clients and publishing NavSatFix ROS 2 topic."""

    def __init__(self, node: Node, topic_name: str = '/robot/gps', frame_id: str = 'gps_link'):
        self.node = node
        self.frame_id = frame_id
        self.publisher = self.node.create_publisher(
            NavSatFix,
            topic_name,
            qos_profile_sensor_data
        )

    def handle_gps(self, data: dict, stamp=None) -> None:
        """Parses GPS JSON payload and publishes NavSatFix ROS 2 message."""
        if stamp is None:
            stamp = self.node.get_clock().now().to_msg()

        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = float(data.get('latitude', 0.0))
        msg.longitude = float(data.get('longitude', 0.0))
        msg.altitude = float(data.get('altitude', 0.0))

        accuracy = float(data.get('accuracy', 0.0))
        var_h = accuracy ** 2

        msg.position_covariance = [
            var_h, 0.0, 0.0,
            0.0, var_h, 0.0,
            0.0, 0.0, 0.0
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.publisher.publish(msg)
