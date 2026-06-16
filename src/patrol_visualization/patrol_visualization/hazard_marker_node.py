import math
import os

import yaml

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from patrol_msgs.msg import CheckRequest
from patrol_msgs.msg import HazardEvent


class HazardMarkerNode(Node):
    def __init__(self):
        super().__init__('hazard_marker_node')

        self.declare_parameter('waypoint_file', 'waypoints.yaml')
        self.declare_parameter('publish_period', 0.5)

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

        self.status_by_id = {
            int(wp['id']): 'pending'
            for wp in self.waypoints
        }

        self.description_by_id = {
            int(wp['id']): ''
            for wp in self.waypoints
        }

        self.current_waypoint_id = None

        self.check_sub = self.create_subscription(
            CheckRequest,
            '/check_request',
            self.check_request_callback,
            10
        )

        self.event_sub = self.create_subscription(
            HazardEvent,
            '/hazard_event',
            self.hazard_event_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/patrol_markers',
            10
        )

        self.timer = self.create_timer(
            self.publish_period,
            self.publish_markers
        )

        self.get_logger().info('Hazard marker node started.')
        self.get_logger().info('Publishing RViz markers to /patrol_markers.')
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')

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

    def check_request_callback(self, msg):
        self.current_waypoint_id = int(msg.waypoint_id)
        self.status_by_id[self.current_waypoint_id] = 'checking'
        self.description_by_id[self.current_waypoint_id] = (
            f'Checking {msg.waypoint_name}: {list(msg.check_items)}'
        )

        self.get_logger().info(
            f'Checking waypoint {msg.waypoint_id} ({msg.waypoint_name})'
        )

    def hazard_event_callback(self, msg):
        waypoint_id = int(msg.waypoint_id)

        if msg.is_abnormal:
            if msg.event_type == 'person_intrusion':
                status = 'person_intrusion'
            elif msg.event_type == 'obstacle_detected':
                status = 'obstacle_detected'
            else:
                status = 'abnormal'
        else:
            status = 'normal'

        self.status_by_id[waypoint_id] = status
        self.description_by_id[waypoint_id] = msg.description

        self.get_logger().info(
            f'Updated marker status: waypoint={waypoint_id}, status={status}'
        )

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        for index, waypoint in enumerate(self.waypoints):
            waypoint_id = int(waypoint['id'])
            waypoint_name = str(waypoint['name'])
            waypoint_type = str(waypoint['type'])

            x = float(waypoint['x'])
            y = float(waypoint['y'])
            yaw = float(waypoint['yaw'])

            status = self.status_by_id.get(waypoint_id, 'pending')

            # Building context markers first: corridor or room
            if waypoint_type == 'passage':
                marker_array.markers.extend(
                    self.create_corridor_markers(
                        index,
                        waypoint_id,
                        waypoint_name,
                        x,
                        y,
                        yaw,
                        now
                    )
                )

            if waypoint_type == 'restricted_area':
                marker_array.markers.extend(
                    self.create_room_markers(
                        index,
                        waypoint_id,
                        waypoint_name,
                        x,
                        y,
                        now
                    )
                )

            marker_array.markers.extend(
                self.create_waypoint_markers(
                    index,
                    waypoint_id,
                    waypoint_name,
                    waypoint_type,
                    x,
                    y,
                    yaw,
                    status,
                    now
                )
            )

            if status == 'person_intrusion':
                marker_array.markers.extend(
                    self.create_person_intrusion_markers(
                        index,
                        waypoint_id,
                        x,
                        y,
                        now
                    )
                )

            if status == 'obstacle_detected':
                marker_array.markers.extend(
                    self.create_obstacle_markers(
                        index,
                        waypoint_id,
                        x,
                        y,
                        yaw,
                        now
                    )
                )

        self.marker_pub.publish(marker_array)

    def create_corridor_markers(self, index, waypoint_id, waypoint_name, x, y, yaw, now):
        markers = []

        corridor = self.make_marker(
            ns='patrol_corridor_area',
            marker_id=10000 + index,
            marker_type=Marker.CUBE,
            x=x,
            y=y,
            z=0.02,
            color=(0.25, 0.55, 1.0, 0.18),
            now=now,
            yaw=yaw
        )
        corridor.scale.x = 1.20
        corridor.scale.y = 0.42
        corridor.scale.z = 0.03
        markers.append(corridor)

        label = self.make_marker(
            ns='patrol_corridor_label',
            marker_id=10100 + index,
            marker_type=Marker.TEXT_VIEW_FACING,
            x=x,
            y=y + 0.28,
            z=0.35,
            color=(0.45, 0.75, 1.0, 1.0),
            now=now
        )
        label.scale.z = 0.17
        label.text = 'Corridor check'
        markers.append(label)

        return markers

    def create_room_markers(self, index, waypoint_id, waypoint_name, x, y, now):
        markers = []

        room = self.make_marker(
            ns='restricted_room_area',
            marker_id=11000 + index,
            marker_type=Marker.CUBE,
            x=x,
            y=y,
            z=0.025,
            color=(1.0, 0.15, 0.15, 0.20),
            now=now
        )
        room.scale.x = 0.85
        room.scale.y = 0.85
        room.scale.z = 0.04
        markers.append(room)

        room_label = self.make_marker(
            ns='restricted_room_label',
            marker_id=11100 + index,
            marker_type=Marker.TEXT_VIEW_FACING,
            x=x,
            y=y + 0.50,
            z=0.45,
            color=(1.0, 0.35, 0.35, 1.0),
            now=now
        )
        room_label.scale.z = 0.18
        room_label.text = 'Restricted room'
        markers.append(room_label)

        return markers

    def create_waypoint_markers(
        self,
        index,
        waypoint_id,
        waypoint_name,
        waypoint_type,
        x,
        y,
        yaw,
        status,
        now
    ):
        markers = []

        color = self.color_for_status(status)

        sphere = self.make_marker(
            ns='patrol_waypoint_points',
            marker_id=index,
            marker_type=Marker.SPHERE,
            x=x,
            y=y,
            z=0.17,
            color=color,
            now=now
        )
        sphere.scale.x = 0.26
        sphere.scale.y = 0.26
        sphere.scale.z = 0.26
        markers.append(sphere)

        arrow = self.make_marker(
            ns='patrol_waypoint_arrows',
            marker_id=1000 + index,
            marker_type=Marker.ARROW,
            x=x,
            y=y,
            z=0.23,
            color=color,
            now=now,
            yaw=yaw
        )
        arrow.scale.x = 0.43
        arrow.scale.y = 0.07
        arrow.scale.z = 0.10
        markers.append(arrow)

        text = self.make_marker(
            ns='patrol_waypoint_labels',
            marker_id=2000 + index,
            marker_type=Marker.TEXT_VIEW_FACING,
            x=x,
            y=y,
            z=0.72,
            color=(1.0, 1.0, 1.0, 1.0),
            now=now
        )
        text.scale.z = 0.20
        text.text = (
            f'{waypoint_id}: {waypoint_name}\n'
            f'{waypoint_type} | {status}'
        )
        markers.append(text)

        return markers

    def create_person_intrusion_markers(self, index, waypoint_id, x, y, now):
        markers = []

        body = self.make_marker(
            ns='person_intrusion_body',
            marker_id=4000 + index,
            marker_type=Marker.CYLINDER,
            x=x,
            y=y,
            z=0.50,
            color=(1.0, 0.0, 0.0, 1.0),
            now=now
        )
        body.scale.x = 0.30
        body.scale.y = 0.30
        body.scale.z = 0.75
        markers.append(body)

        head = self.make_marker(
            ns='person_intrusion_head',
            marker_id=4100 + index,
            marker_type=Marker.SPHERE,
            x=x,
            y=y,
            z=0.98,
            color=(1.0, 0.2, 0.2, 1.0),
            now=now
        )
        head.scale.x = 0.32
        head.scale.y = 0.32
        head.scale.z = 0.32
        markers.append(head)

        alert_ring = self.make_marker(
            ns='person_intrusion_alert_zone',
            marker_id=4150 + index,
            marker_type=Marker.CYLINDER,
            x=x,
            y=y,
            z=0.04,
            color=(1.0, 0.0, 0.0, 0.35),
            now=now
        )
        alert_ring.scale.x = 0.95
        alert_ring.scale.y = 0.95
        alert_ring.scale.z = 0.04
        markers.append(alert_ring)

        label = self.make_marker(
            ns='person_intrusion_label',
            marker_id=4200 + index,
            marker_type=Marker.TEXT_VIEW_FACING,
            x=x,
            y=y,
            z=1.35,
            color=(1.0, 0.0, 0.0, 1.0),
            now=now
        )
        label.scale.z = 0.24
        label.text = 'PERSON\nINTRUSION'
        markers.append(label)

        return markers

    def create_obstacle_markers(self, index, waypoint_id, x, y, yaw, now):
        markers = []

        obstacle_x = x + 1.40 * math.cos(yaw)
        obstacle_y = y + 1.40 * math.sin(yaw)

        box = self.make_marker(
            ns='detected_obstacle_box',
            marker_id=5000 + index,
            marker_type=Marker.CUBE,
            x=obstacle_x,
            y=obstacle_y,
            z=0.28,
            color=(1.0, 0.35, 0.0, 1.0),
            now=now
        )
        box.scale.x = 0.42
        box.scale.y = 0.42
        box.scale.z = 0.55
        markers.append(box)

        label = self.make_marker(
            ns='detected_obstacle_label',
            marker_id=5100 + index,
            marker_type=Marker.TEXT_VIEW_FACING,
            x=obstacle_x,
            y=obstacle_y,
            z=0.88,
            color=(1.0, 0.5, 0.0, 1.0),
            now=now
        )
        label.scale.z = 0.23
        label.text = 'OBSTACLE'
        markers.append(label)

        return markers

    def color_for_status(self, status):
        if status == 'checking':
            return (1.0, 1.0, 0.0, 1.0)

        if status == 'normal':
            return (0.0, 1.0, 0.0, 1.0)

        if status == 'person_intrusion':
            return (1.0, 0.0, 0.0, 1.0)

        if status == 'obstacle_detected':
            return (1.0, 0.45, 0.0, 1.0)

        if status == 'abnormal':
            return (1.0, 0.0, 1.0, 1.0)

        return (0.45, 0.45, 0.45, 1.0)

    def make_marker(self, ns, marker_id, marker_type, x, y, z, color, now, yaw=0.0):
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = now

        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)

        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)

        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])

        marker.lifetime = Duration(sec=0)

        return marker


def main(args=None):
    rclpy.init(args=args)

    node = HazardMarkerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Hazard marker node stopped.')
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()