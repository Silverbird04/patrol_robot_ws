import math

import cv2
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

from patrol_msgs.msg import CheckRequest
from patrol_msgs.msg import HazardEvent


class DummyHazardDetector(Node):
    def __init__(self):
        super().__init__('dummy_hazard_detector')

        self.bridge = CvBridge()

        self.latest_request = None
        self.pending_check = False
        self.frame_count = 0

        self.latest_scan = None
        self.front_min_distance = None

        self.last_event = None
        self.last_result_text = 'WAITING FOR CHECK REQUEST'
        self.last_result_color = (0, 255, 255)  # yellow

        self.obstacle_distance_threshold = 0.8
        self.front_angle_deg = 15.0

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.check_sub = self.create_subscription(
            CheckRequest,
            '/check_request',
            self.check_request_callback,
            10
        )

        self.event_pub = self.create_publisher(
            HazardEvent,
            '/hazard_event',
            10
        )

        self.get_logger().info('Dummy hazard detector started.')
        self.get_logger().info('Subscribing to /camera/image_raw, /scan, and /check_request.')
        self.get_logger().info(
            f'Obstacle rule: front +/- {self.front_angle_deg:.1f} deg, '
            f'distance < {self.obstacle_distance_threshold:.2f} m'
        )

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.front_min_distance = self.compute_front_min_distance(msg)

    def check_request_callback(self, msg):
        self.latest_request = msg
        self.pending_check = True

        self.last_result_text = 'CHECK REQUEST RECEIVED'
        self.last_result_color = (0, 255, 255)  # yellow

        self.get_logger().info(
            f'Received check request: '
            f'id={msg.waypoint_id}, '
            f'name={msg.waypoint_name}, '
            f'type={msg.waypoint_type}, '
            f'items={list(msg.check_items)}'
        )

    def image_callback(self, msg):
        self.frame_count += 1

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        height, width, _ = frame.shape
        roi = self.get_roi(width, height)

        if self.pending_check and self.latest_request is not None:
            event_msg = self.perform_check(self.latest_request)
            self.event_pub.publish(event_msg)
            self.last_event = event_msg

            if event_msg.is_abnormal:
                self.last_result_text = self.make_abnormal_result_text(event_msg)
                self.last_result_color = (0, 0, 255)  # red

                self.get_logger().warn(
                    f'Published hazard event: '
                    f'{event_msg.event_type}, '
                    f'waypoint={event_msg.waypoint_id}'
                )
            else:
                self.last_result_text = self.make_normal_result_text(event_msg)
                self.last_result_color = (0, 255, 0)  # green

                self.get_logger().info(
                    f'Published normal event: '
                    f'{event_msg.event_type}, '
                    f'waypoint={event_msg.waypoint_id}'
                )

            self.pending_check = False

        self.draw_dashboard(frame)
        self.draw_roi(frame, roi)

        cv2.imshow('Patrol Hazard Detector', frame)
        cv2.waitKey(1)

    def get_roi(self, width, height):
        x1 = int(width * 0.35)
        y1 = int(height * 0.30)
        x2 = int(width * 0.65)
        y2 = int(height * 0.70)

        return x1, y1, x2, y2

    def draw_roi(self, frame, roi):
        x1, y1, x2, y2 = roi

        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            self.last_result_color,
            2
        )

        cv2.putText(
            frame,
            'Inspection ROI',
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            self.last_result_color,
            2
        )

    def draw_dashboard(self, frame):
        panel_x = 20
        panel_y = 20
        line_h = 28

        cv2.rectangle(
            frame,
            (10, 10),
            (620, 215),
            (0, 0, 0),
            -1
        )

        cv2.rectangle(
            frame,
            (10, 10),
            (620, 215),
            (255, 255, 255),
            1
        )

        self.put_text(
            frame,
            'Patrol Hazard Detector',
            panel_x,
            panel_y + line_h,
            (255, 255, 255),
            0.7,
            2
        )

        if self.latest_request is None:
            self.put_text(
                frame,
                'Current waypoint: none',
                panel_x,
                panel_y + line_h * 2,
                (200, 200, 200)
            )
            self.put_text(
                frame,
                'Waiting for /check_request...',
                panel_x,
                panel_y + line_h * 3,
                (0, 255, 255)
            )
        else:
            req = self.latest_request

            self.put_text(
                frame,
                f'Waypoint {req.waypoint_id}: {req.waypoint_name}',
                panel_x,
                panel_y + line_h * 2,
                (255, 255, 255)
            )

            self.put_text(
                frame,
                f'Mode: {req.waypoint_type} | Items: {list(req.check_items)}',
                panel_x,
                panel_y + line_h * 3,
                (255, 255, 255)
            )

            self.put_text(
                frame,
                f'Position: x={req.x:.2f}, y={req.y:.2f}, yaw={req.yaw:.2f}',
                panel_x,
                panel_y + line_h * 4,
                (255, 255, 255)
            )

        self.put_text(
            frame,
            f'Front LiDAR: {self.make_scan_text()}',
            panel_x,
            panel_y + line_h * 5,
            (255, 255, 255)
        )

        self.put_text(
            frame,
            f'Result: {self.last_result_text}',
            panel_x,
            panel_y + line_h * 6,
            self.last_result_color,
            0.65,
            2
        )

    def put_text(self, frame, text, x, y, color, scale=0.6, thickness=1):
        cv2.putText(
            frame,
            text,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness
        )

    def make_scan_text(self):
        if self.latest_scan is None:
            return 'waiting for /scan'

        if self.front_min_distance is None:
            return 'no valid object in front sector'

        return f'{self.front_min_distance:.2f} m'

    def compute_front_min_distance(self, scan_msg):
        front_angle_rad = math.radians(self.front_angle_deg)

        valid_ranges = []

        for i, distance in enumerate(scan_msg.ranges):
            angle = scan_msg.angle_min + i * scan_msg.angle_increment

            if abs(angle) > front_angle_rad:
                continue

            if math.isnan(distance) or math.isinf(distance):
                continue

            if distance < scan_msg.range_min or distance > scan_msg.range_max:
                continue

            valid_ranges.append(distance)

        if not valid_ranges:
            return None

        return min(valid_ranges)

    def perform_check(self, request):
        msg = HazardEvent()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.waypoint_id = request.waypoint_id
        msg.waypoint_type = request.waypoint_type

        msg.x = request.x
        msg.y = request.y
        msg.yaw = request.yaw

        # 1. 출입 금지 구역 사람 침입 판단
        # 현재는 restricted_area + person 점검 요청이면 침입으로 가정한다.
        # 나중에 실제 person detector 또는 Gazebo actor 기반 판단으로 확장 가능.
        if (
            request.waypoint_type == 'restricted_area'
            and 'person' in request.check_items
        ):
            msg.event_type = 'person_intrusion'
            msg.is_abnormal = True
            msg.description = (
                f'Person intrusion detected at restricted area waypoint '
                f'{request.waypoint_id} ({request.waypoint_name})'
            )

            return msg

        # 2. 주요 통로 장애물 판단
        # passage + obstacle 점검 요청이면 /scan 전방 거리로 장애물 여부를 판단한다.
        if (
            request.waypoint_type == 'passage'
            and 'obstacle' in request.check_items
        ):
            if self.is_front_obstacle_detected():
                msg.event_type = 'obstacle_detected'
                msg.is_abnormal = True

                if self.front_min_distance is None:
                    distance_text = 'unknown distance'
                else:
                    distance_text = f'{self.front_min_distance:.2f} m'

                msg.description = (
                    f'Obstacle detected in front passage at waypoint '
                    f'{request.waypoint_id} ({request.waypoint_name}), '
                    f'distance={distance_text}'
                )

                return msg

        # 3. 이상 없음
        msg.event_type = 'normal_check'
        msg.is_abnormal = False
        msg.description = (
            f'No hazard detected at waypoint '
            f'{request.waypoint_id} ({request.waypoint_name})'
        )

        return msg

    def is_front_obstacle_detected(self):
        if self.front_min_distance is None:
            return False

        return self.front_min_distance < self.obstacle_distance_threshold

    def make_abnormal_result_text(self, event_msg):
        if event_msg.event_type == 'person_intrusion':
            return 'ABNORMAL - PERSON INTRUSION'

        if event_msg.event_type == 'obstacle_detected':
            return 'ABNORMAL - OBSTACLE'

        return f'ABNORMAL - {event_msg.event_type}'

    def make_normal_result_text(self, event_msg):
        return 'NORMAL - NO HAZARD'


def main(args=None):
    rclpy.init(args=args)

    node = DummyHazardDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Dummy hazard detector stopped.')
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()