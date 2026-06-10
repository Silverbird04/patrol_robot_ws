import csv
import json
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class EventLogger(Node):
    def __init__(self):
        super().__init__('event_logger')

        self.subscription = self.create_subscription(
            String,
            '/hazard_event',
            self.event_callback,
            10
        )

        self.log_dir = Path.home() / 'patrol_robot_ws' / 'logs'
        self.log_dir.mkdir(parents=True, exist_ok=True)

        self.log_path = self.log_dir / 'patrol_log.csv'

        self.prepare_csv_file()

        self.get_logger().info(f'Event logger started.')
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
                    'is_abnormal',
                    'description'
                ])

    def event_callback(self, msg):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON message: {msg.data}')
            return

        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        event_type = event.get('event_type', 'unknown')
        waypoint_id = event.get('waypoint_id', -1)
        waypoint_type = event.get('waypoint_type', 'unknown')
        is_abnormal = event.get('is_abnormal', False)
        description = event.get('description', '')

        with open(self.log_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                now,
                event_type,
                waypoint_id,
                waypoint_type,
                is_abnormal,
                description
            ])

        if is_abnormal:
            self.get_logger().warn(
                f'Logged abnormal event: {event_type}, waypoint={waypoint_id}, description={description}'
            )
        else:
            self.get_logger().info(
                f'Logged normal event: {event_type}, waypoint={waypoint_id}'
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