"""Battery Sensor Bridge module for ROS 2."""

from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from rclpy.qos import qos_profile_sensor_data


class BatteryBridge:
    """Handles parsing battery data from web clients and publishing BatteryState ROS 2 topic."""

    def __init__(self, node: Node, topic_name: str = '/robot/battery', frame_id: str = 'phone_link'):
        self.node = node
        self.frame_id = frame_id
        self.publisher = self.node.create_publisher(
            BatteryState,
            topic_name,
            qos_profile_sensor_data
        )

    def handle_battery(self, data: dict, stamp=None) -> None:
        """Parses battery JSON payload and publishes BatteryState ROS 2 message."""
        if stamp is None:
            stamp = self.node.get_clock().now().to_msg()

        msg = BatteryState()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        # Browser Battery Status API level is float from 0.0 to 1.0
        level = float(data.get('level', 1.0))
        msg.percentage = level

        is_charging = bool(data.get('charging', False))
        if is_charging:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        msg.present = True

        msg.voltage = float('nan')
        msg.temperature = float('nan')
        msg.current = float('nan')
        msg.charge = float('nan')
        msg.capacity = float('nan')
        msg.design_capacity = float('nan')

        self.publisher.publish(msg)
