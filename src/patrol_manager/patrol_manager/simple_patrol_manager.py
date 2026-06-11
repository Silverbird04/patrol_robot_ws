import os

import yaml

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from patrol_msgs.msg import CheckRequest


class SimplePatrolManager(Node):
    def __init__(self):
        super().__init__('simple_patrol_manager')

        self.check_pub = self.create_publisher(
            CheckRequest,
            '/check_request',
            10
        )

        self.declare_parameter('waypoint_file', 'waypoints.yaml')
        self.declare_parameter('publish_period', 8.0)

        waypoint_file_name = (
            self.get_parameter('waypoint_file')
            .get_parameter_value()
            .string_value
        )

        self.publish_period = (
            self.get_parameter('publish_period')
            .get_parameter_value()
            .double_value
        )

        self.waypoints = self.load_waypoints(waypoint_file_name)
        self.current_index = 0

        self.timer = self.create_timer(
            self.publish_period,
            self.publish_next_check_request
        )

        self.get_logger().info('Simple patrol manager started.')
        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints from {waypoint_file_name}'
        )
        self.get_logger().info(
            f'Publishing waypoint check requests to /check_request every {self.publish_period} seconds.'
        )

    def load_waypoints(self, waypoint_file_name):
        package_share = get_package_share_directory('patrol_manager')

        waypoint_path = os.path.join(
            package_share,
            'config',
            waypoint_file_name
        )

        if not os.path.exists(waypoint_path):
            raise FileNotFoundError(f'Waypoint file not found: {waypoint_path}')

        with open(waypoint_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        waypoints = data.get('waypoints', [])

        if not waypoints:
            raise ValueError('No waypoints found in waypoint file.')

        self.get_logger().info(f'Waypoint file path: {waypoint_path}')

        return waypoints

    def publish_next_check_request(self):
        waypoint = self.waypoints[self.current_index]

        msg = CheckRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.waypoint_id = int(waypoint['id'])
        msg.waypoint_name = str(waypoint['name'])
        msg.waypoint_type = str(waypoint['type'])

        msg.x = float(waypoint['x'])
        msg.y = float(waypoint['y'])
        msg.yaw = float(waypoint['yaw'])

        msg.check_items = [str(item) for item in waypoint['check_items']]

        self.check_pub.publish(msg)

        self.get_logger().info(
            f'Published check request: '
            f'id={msg.waypoint_id}, '
            f'name={msg.waypoint_name}, '
            f'type={msg.waypoint_type}, '
            f'position=({msg.x:.2f}, {msg.y:.2f}, {msg.yaw:.2f}), '
            f'items={list(msg.check_items)}'
        )

        self.current_index = (self.current_index + 1) % len(self.waypoints)


def main(args=None):
    rclpy.init(args=args)

    node = SimplePatrolManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Simple patrol manager stopped.')
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()