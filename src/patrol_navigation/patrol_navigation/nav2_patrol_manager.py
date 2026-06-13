import math
import os

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from ament_index_python.packages import get_package_share_directory

from patrol_msgs.msg import CheckRequest


class Nav2PatrolManager(Node):
    def __init__(self):
        super().__init__('nav2_patrol_manager')

        self.declare_parameter('waypoint_file', 'waypoints.yaml')
        self.declare_parameter('check_wait_time', 4.0)
        self.declare_parameter('loop_patrol', False)

        waypoint_file_name = (
            self.get_parameter('waypoint_file')
            .get_parameter_value()
            .string_value
        )

        self.check_wait_time = (
            self.get_parameter('check_wait_time')
            .get_parameter_value()
            .double_value
        )

        self.loop_patrol = (
            self.get_parameter('loop_patrol')
            .get_parameter_value()
            .bool_value
        )

        self.waypoints = self.load_waypoints(waypoint_file_name)
        self.current_index = 0
        self.wait_timer = None

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.check_pub = self.create_publisher(
            CheckRequest,
            '/check_request',
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/waypoints',
            10
        )

        self.get_logger().info('Nav2 patrol manager started.')
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')
        self.get_logger().info('Waiting for navigate_to_pose action server...')

        self.nav_client.wait_for_server()

        self.get_logger().info('navigate_to_pose action server is ready.')
        self.publish_waypoint_markers()
        self.send_next_goal()

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

    def send_next_goal(self):
        if self.current_index >= len(self.waypoints):
            if self.loop_patrol:
                self.get_logger().info('Loop patrol enabled. Restarting from waypoint 1.')
                self.current_index = 0
            else:
                self.get_logger().info('All waypoints completed.')
                return

        waypoint = self.waypoints[self.current_index]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose_stamped(
            float(waypoint['x']),
            float(waypoint['y']),
            float(waypoint['yaw'])
        )

        self.get_logger().info(
            f'[{self.current_index + 1}/{len(self.waypoints)}] '
            f'Navigating to waypoint {waypoint["id"]} ({waypoint["name"]}) '
            f'at x={float(waypoint["x"]):.2f}, '
            f'y={float(waypoint["y"]):.2f}, '
            f'yaw={float(waypoint["yaw"]):.2f}'
        )

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def make_pose_stamped(self, x, y, yaw_rad):
        pose = PoseStamped()

        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y

        pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        return pose

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn(
                f'Goal rejected at waypoint index {self.current_index + 1}.'
            )
            return

        self.get_logger().info('Goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        result = future.result()
        status = result.status

        waypoint = self.waypoints[self.current_index]

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Waypoint {waypoint["id"]} ({waypoint["name"]}) reached.'
            )

            self.publish_check_request(waypoint)

            self.get_logger().info(
                f'Waiting {self.check_wait_time:.1f} seconds for inspection result.'
            )

            self.wait_timer = self.create_timer(
                self.check_wait_time,
                self.after_check_wait
            )

        else:
            self.get_logger().warn(
                f'Navigation failed at waypoint {waypoint["id"]} '
                f'({waypoint["name"]}), status={status}.'
            )

    def after_check_wait(self):
        if self.wait_timer is not None:
            self.wait_timer.cancel()
            self.wait_timer = None

        self.current_index += 1
        self.publish_waypoint_markers()
        self.send_next_goal()

    def publish_check_request(self, waypoint):
        msg = CheckRequest()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.waypoint_id = int(waypoint['id'])
        msg.waypoint_name = str(waypoint['name'])
        msg.waypoint_type = str(waypoint['type'])

        msg.x = float(waypoint['x'])
        msg.y = float(waypoint['y'])
        msg.yaw = float(waypoint['yaw'])

        msg.check_items = [
            str(item) for item in waypoint.get('check_items', [])
        ]

        self.check_pub.publish(msg)

        self.get_logger().info(
            f'Published check request: '
            f'id={msg.waypoint_id}, '
            f'name={msg.waypoint_name}, '
            f'type={msg.waypoint_type}, '
            f'items={list(msg.check_items)}'
        )

    def publish_waypoint_markers(self):
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        for i, waypoint in enumerate(self.waypoints):
            x = float(waypoint['x'])
            y = float(waypoint['y'])
            yaw = float(waypoint['yaw'])

            if i < self.current_index:
                color = (0.5, 0.5, 0.5)
            elif i == self.current_index:
                color = (0.0, 0.9, 0.0)
            else:
                color = (1.0, 0.6, 0.0)

            sphere = self.make_marker(
                ns='patrol_waypoints',
                marker_id=i,
                marker_type=Marker.SPHERE,
                x=x,
                y=y,
                z=0.15,
                color=color,
                now=now
            )
            sphere.scale.x = 0.25
            sphere.scale.y = 0.25
            sphere.scale.z = 0.25

            arrow = self.make_marker(
                ns='patrol_waypoint_arrows',
                marker_id=i,
                marker_type=Marker.ARROW,
                x=x,
                y=y,
                z=0.15,
                color=color,
                now=now
            )
            arrow.pose.orientation.z = math.sin(yaw / 2.0)
            arrow.pose.orientation.w = math.cos(yaw / 2.0)
            arrow.scale.x = 0.35
            arrow.scale.y = 0.06
            arrow.scale.z = 0.08

            label = self.make_marker(
                ns='patrol_waypoint_labels',
                marker_id=i,
                marker_type=Marker.TEXT_VIEW_FACING,
                x=x,
                y=y,
                z=0.45,
                color=(1.0, 1.0, 1.0),
                now=now
            )
            label.scale.z = 0.25
            label.text = f'{waypoint["id"]}: {waypoint["name"]}'

            marker_array.markers.extend([
                sphere,
                arrow,
                label
            ])

        self.marker_pub.publish(marker_array)

    def make_marker(self, ns, marker_id, marker_type, x, y, z, color, now):
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = now

        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0

        marker.lifetime = Duration(sec=0)

        return marker


def main(args=None):
    rclpy.init(args=args)

    node = Nav2PatrolManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nav2 patrol manager stopped.')
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()