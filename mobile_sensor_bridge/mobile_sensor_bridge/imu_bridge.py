"""IMU bridge module for handling smartphone IMU sensor data publishing in ROS2."""

import math
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu


class ImuBridge:
    """Handles IMU sensor topic publishing and coordinate/unit transformations."""

    def __init__(self, node):
        """Initializes publisher for IMU topic.

        Args:
            node (rclpy.node.Node): Parent ROS2 node instance.
        """
        self.node = node
        self.imu_publisher = self.node.create_publisher(
            Imu,
            '/imu/data_raw',
            10
        )

    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        """Converts Euler angles (roll, pitch, yaw in radians) to a ROS2 Quaternion message."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def handle_imu(self, data: dict, stamp) -> None:
        """Processes IMU dictionary payload received over HTTP and publishes to ROS2 IMU topic.

        Args:
            data (dict): IMU sensor dictionary containing accel, gyro, orientation.
            stamp (builtin_interfaces.msg.Time): ROS2 time stamp.
        """
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = 'phone_imu'

        # Linear Acceleration (m/s^2)
        accel = data.get('accel', {})
        imu_msg.linear_acceleration.x = float(accel.get('x', 0.0))
        imu_msg.linear_acceleration.y = float(accel.get('y', 0.0))
        imu_msg.linear_acceleration.z = float(accel.get('z', 0.0))

        # Angular Velocity (deg/s -> rad/s)
        gyro = data.get('gyro', {})
        imu_msg.angular_velocity.x = math.radians(float(gyro.get('beta', 0.0)))   # pitch rate
        imu_msg.angular_velocity.y = math.radians(float(gyro.get('gamma', 0.0)))  # roll rate
        imu_msg.angular_velocity.z = math.radians(float(gyro.get('alpha', 0.0)))  # yaw rate

        # Orientation (deg -> rad -> Quaternion)
        ori = data.get('orientation', {})
        alpha_rad = math.radians(float(ori.get('alpha', 0.0)))  # yaw [0, 360]
        beta_rad = math.radians(float(ori.get('beta', 0.0)))    # pitch [-180, 180]
        gamma_rad = math.radians(float(ori.get('gamma', 0.0)))  # roll [-90, 90]

        imu_msg.orientation = self.euler_to_quaternion(gamma_rad, beta_rad, alpha_rad)

        self.imu_publisher.publish(imu_msg)
