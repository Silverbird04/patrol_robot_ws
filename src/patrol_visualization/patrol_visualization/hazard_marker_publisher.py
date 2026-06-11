import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from patrol_msgs.msg import HazardEvent


class HazardMarkerPublisher(Node):
    def __init__(self):
        super().__init__('hazard_marker_publisher')

        self.event_sub = self.create_subscription(
            HazardEvent,
            '/hazard_event',
            self.event_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/patrol_markers',
            10
        )

        self.get_logger().info('Hazard marker publisher started.')
        self.get_logger().info('Listening to /hazard_event and publishing /patrol_markers.')

    def event_callback(self, msg):
        marker_array = MarkerArray()

        status_marker = self.create_status_marker(msg)
        text_marker = self.create_text_marker(msg)

        marker_array.markers.append(status_marker)
        marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

        if msg.is_abnormal:
            self.get_logger().warn(
                f'Published RED hazard marker: waypoint={msg.waypoint_id}, event={msg.event_type}'
            )
        else:
            self.get_logger().info(
                f'Published GREEN normal marker: waypoint={msg.waypoint_id}, event={msg.event_type}'
            )

    def create_status_marker(self, event_msg):
        marker = Marker()

        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'

        marker.ns = 'patrol_status'
        marker.id = event_msg.waypoint_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = event_msg.x
        marker.pose.position.y = event_msg.y
        marker.pose.position.z = 0.15

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35

        if event_msg.is_abnormal:
            # 이상 상황: red
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.9
        else:
            # 정상 점검: green
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.9

        return marker

    def create_text_marker(self, event_msg):
        marker = Marker()

        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'

        marker.ns = 'patrol_text'
        marker.id = event_msg.waypoint_id + 1000
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = event_msg.x
        marker.pose.position.y = event_msg.y
        marker.pose.position.z = 0.55

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.25

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        status_text = 'ABNORMAL' if event_msg.is_abnormal else 'NORMAL'

        marker.text = (
            f'WP {event_msg.waypoint_id}\n'
            f'{event_msg.event_type}\n'
            f'{status_text}'
        )

        return marker


def main(args=None):
    rclpy.init(args=args)

    node = HazardMarkerPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Hazard marker publisher stopped.')
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()