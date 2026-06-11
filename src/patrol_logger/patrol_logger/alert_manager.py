import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from patrol_msgs.msg import HazardEvent


class AlertManager(Node):
    def __init__(self):
        super().__init__('alert_manager')

        self.alert_sub = self.create_subscription(
            HazardEvent,
            '/hazard_event',
            self.hazard_event_callback,
            10
        )

        self.log_dir = os.path.expanduser('~/patrol_robot_ws/logs')
        os.makedirs(self.log_dir, exist_ok=True)

        self.alert_log_path = os.path.join(
            self.log_dir,
            'alert_log.txt'
        )

        self.get_logger().info('Alert manager started.')
        self.get_logger().info('Subscribing to /hazard_event.')
        self.get_logger().info(f'Alert log path: {self.alert_log_path}')

    def hazard_event_callback(self, msg):
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        if msg.is_abnormal:
            alert_text = (
                f'[ALERT] {current_time} | '
                f'waypoint={msg.waypoint_id} | '
                f'type={msg.waypoint_type} | '
                f'event={msg.event_type} | '
                f'position=({msg.x:.2f}, {msg.y:.2f}, {msg.yaw:.2f}) | '
                f'{msg.description}'
            )

            self.get_logger().warn(alert_text)
            self.write_alert_log(alert_text)

        else:
            normal_text = (
                f'[NORMAL] {current_time} | '
                f'waypoint={msg.waypoint_id} | '
                f'type={msg.waypoint_type} | '
                f'event={msg.event_type} | '
                f'position=({msg.x:.2f}, {msg.y:.2f}, {msg.yaw:.2f}) | '
                f'{msg.description}'
            )

            self.get_logger().info(normal_text)
            self.write_alert_log(normal_text)

    def write_alert_log(self, text):
        with open(self.alert_log_path, 'a', encoding='utf-8') as f:
            f.write(text + '\n')


def main(args=None):
    rclpy.init(args=args)

    node = AlertManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Alert manager stopped.')
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()