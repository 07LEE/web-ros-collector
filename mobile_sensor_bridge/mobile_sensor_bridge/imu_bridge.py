"""IMU bridge module for handling smartphone IMU sensor data publishing in ROS2."""

import math
from geometry_msgs.msg import Quaternion
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuBridge:
    """Handles IMU sensor topic publishing and coordinate/unit transformations."""

    def __init__(self, node, topic_name: str = 'imu/data_raw', frame_id: str = 'phone_imu'):
        """Initializes publisher for IMU topic.

        Args:
            node (rclpy.node.Node): Parent ROS2 node instance.
            topic_name (str): Topic name to publish IMU messages.
            frame_id (str): Frame ID for header.
        """
        self.node = node
        self.frame_id = frame_id
        self.imu_publisher = self.node.create_publisher(
            Imu,
            topic_name,
            qos_profile_sensor_data
        )

    @staticmethod
    def device_orientation_to_quaternion(alpha: float, beta: float, gamma: float) -> Quaternion:
        """Converts W3C DeviceOrientation (intrinsic Z-X'-Y'' in degrees) to ROS Quaternion."""
        _z = math.radians(alpha)
        _x = math.radians(beta)
        _y = math.radians(gamma)

        cZ = math.cos(_z / 2.0)
        sZ = math.sin(_z / 2.0)
        cX = math.cos(_x / 2.0)
        sX = math.sin(_x / 2.0)
        cY = math.cos(_y / 2.0)
        sY = math.sin(_y / 2.0)

        # W3C Intrinsic Z-X'-Y''
        w_w3c = cZ * cX * cY - sZ * sX * sY
        x_w3c = cZ * sX * cY - sZ * cX * sY
        y_w3c = cZ * cX * sY + sZ * sX * cY
        z_w3c = sZ * cX * cY + cZ * sX * sY

        # Post-multiply by q_z(+90 deg) = (w=sqrt(0.5), x=0, y=0, z=sqrt(0.5)) to align with ROS REP-103
        inv_sqrt2 = 0.7071067811865475
        w = inv_sqrt2 * (w_w3c - z_w3c)
        x = inv_sqrt2 * (x_w3c + y_w3c)
        y = inv_sqrt2 * (y_w3c - x_w3c)
        z = inv_sqrt2 * (z_w3c + w_w3c)

        q = Quaternion()
        q.x = float(x)
        q.y = float(y)
        q.z = float(z)
        q.w = float(w)
        return q

    def _publish_single_imu(self, data: dict, stamp) -> None:
        imu_msg = Imu()

        # Custom sample timestamp if provided in batch
        sample_ts_ms = data.get('clientTimestampMs')
        if sample_ts_ms is not None and hasattr(self.node, 'parse_stamp'):
            imu_msg.header.stamp = self.node.parse_stamp(str(sample_ts_ms))
        else:
            imu_msg.header.stamp = stamp

        imu_msg.header.frame_id = self.frame_id

        # Linear Acceleration (m/s^2) - REP-103 mapping from phone frame (X: right, Y: top, Z: out)
        accel = data.get('accel', {})
        raw_ax = float(accel.get('x', 0.0))
        raw_ay = float(accel.get('y', 0.0))
        raw_az = float(accel.get('z', 0.0))

        # Map to ROS REP-103: X forward (top of phone), Y left, Z up (out of screen)
        imu_msg.linear_acceleration.x = raw_ay
        imu_msg.linear_acceleration.y = -raw_ax
        imu_msg.linear_acceleration.z = raw_az

        # Angular Velocity (deg/s -> rad/s) - REP-103 mapping
        gyro = data.get('gyro', {})
        raw_ga = float(gyro.get('alpha', 0.0))  # rate around Z
        raw_gb = float(gyro.get('beta', 0.0))   # rate around X
        raw_gg = float(gyro.get('gamma', 0.0))  # rate around Y

        imu_msg.angular_velocity.x = math.radians(raw_gg)
        imu_msg.angular_velocity.y = -math.radians(raw_gb)
        imu_msg.angular_velocity.z = math.radians(raw_ga)

        # Orientation (W3C intrinsic Z-X'-Y'' in deg)
        ori = data.get('orientation', {})
        if ori:
            alpha = float(ori.get('alpha', 0.0))
            beta = float(ori.get('beta', 0.0))
            gamma = float(ori.get('gamma', 0.0))
            imu_msg.orientation = self.device_orientation_to_quaternion(alpha, beta, gamma)
            imu_msg.orientation_covariance = [
                1e-4, 0.0, 0.0,
                0.0, 1e-4, 0.0,
                0.0, 0.0, 1e-4
            ]
        else:
            imu_msg.orientation_covariance[0] = -1.0

        imu_msg.angular_velocity_covariance = [
            1e-4, 0.0, 0.0,
            0.0, 1e-4, 0.0,
            0.0, 0.0, 1e-4
        ]
        imu_msg.linear_acceleration_covariance = [
            1e-2, 0.0, 0.0,
            0.0, 1e-2, 0.0,
            0.0, 0.0, 1e-2
        ]

        self.imu_publisher.publish(imu_msg)

    def handle_imu(self, data, stamp) -> None:
        """Processes IMU payload (dict or list of dicts) received over HTTP and publishes to ROS2 IMU topic."""
        if isinstance(data, list):
            for sample in data:
                self._publish_single_imu(sample, stamp)
        elif isinstance(data, dict):
            self._publish_single_imu(data, stamp)

