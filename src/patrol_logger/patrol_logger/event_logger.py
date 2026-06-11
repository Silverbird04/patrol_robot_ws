
import csv
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node

from patrol_msgs.msg import HazardEvent


class EventLogger(Node):
    def __init__(self):
        super().__init__('event_logger')

        self.subscription = self.create_subscription(
            HazardEvent,
            '/hazard_event',
            self.event_callback,
            10
        )

        self.log_dir = Path.home() / 'patrol_robot_ws' / 'logs'
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.log_path = self.log_dir / 'patrol_log.csv'

        self.prepare_csv_file()

        self.get_logger().info('Event logger started.')
        self.get_logger().info(f'Logging to: {self.log_path}')

    def prepare_csv_file(self):
        if not self.log_path.exists():
            with open(self.log_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'time',
                    'event_type',
                    'waypoint_id',
                    'waypoint_type',
                    'x',
                    'y',
                    'yaw',
                    'is_abnormal',
                    'description'
                ])

    def event_callback(self, msg):
        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        with open(self.log_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                now,
                msg.event_type,
                msg.waypoint_id,
                msg.waypoint_type,
                msg.x,
                msg.y,
                msg.yaw,
                msg.is_abnormal,
                msg.description
            ])

        if msg.is_abnormal:
            self.get_logger().warn(
                f'Logged abnormal event: {msg.event_type}, waypoint={msg.waypoint_id}, description={msg.description}'
            )
        else:
            self.get_logger().info(
                f'Logged normal event: {msg.event_type}, waypoint={msg.waypoint_id}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = EventLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Event logger stopped.')
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
