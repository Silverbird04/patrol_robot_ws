
import cv2
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from patrol_msgs.msg import CheckRequest
from patrol_msgs.msg import HazardEvent


class DummyHazardDetector(Node):
    def __init__(self):
        super().__init__('dummy_hazard_detector')

        self.bridge = CvBridge()

        self.latest_request = None
        self.pending_check = False
        self.frame_count = 0

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
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
        self.get_logger().info('Waiting for /check_request.')

    def check_request_callback(self, msg):
        self.latest_request = msg
        self.pending_check = True

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

        x1 = int(width * 0.35)
        y1 = int(height * 0.30)
        x2 = int(width * 0.65)
        y2 = int(height * 0.70)

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.putText(
            frame,
            'Inspection ROI',
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2
        )

        if self.latest_request is not None:
            label = (
                f'Current request: '
                f'{self.latest_request.waypoint_id} '
                f'{self.latest_request.waypoint_type}'
            )
            cv2.putText(
                frame,
                label,
                (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

        if self.pending_check and self.latest_request is not None:
            event_msg = self.perform_dummy_check(self.latest_request)
            self.event_pub.publish(event_msg)

            if event_msg.is_abnormal:
                cv2.putText(
                    frame,
                    'DUMMY HAZARD DETECTED',
                    (30, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2
                )

                self.get_logger().warn(
                    f'Published hazard event: '
                    f'{event_msg.event_type}, '
                    f'waypoint={event_msg.waypoint_id}'
                )
            else:
                cv2.putText(
                    frame,
                    'NORMAL CHECK',
                    (30, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

                self.get_logger().info(
                    f'Published normal event: '
                    f'{event_msg.event_type}, '
                    f'waypoint={event_msg.waypoint_id}'
                )

            self.pending_check = False

        cv2.imshow('Dummy Hazard Detector', frame)
        cv2.waitKey(1)

    def perform_dummy_check(self, request):
        msg = HazardEvent()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.waypoint_id = request.waypoint_id
        msg.waypoint_type = request.waypoint_type

        msg.x = request.x
        msg.y = request.y
        msg.yaw = request.yaw

        # 임시 판단 규칙:
        # restricted_area에서 person 점검이면 이상 상황으로 가정
        # passage에서 obstacle 점검이면 정상으로 가정
        if request.waypoint_type == 'restricted_area' and 'person' in request.check_items:
            msg.event_type = 'dummy_hazard'
            msg.is_abnormal = True
            msg.description = (
                f'Dummy person intrusion detected at waypoint '
                f'{request.waypoint_id} ({request.waypoint_name})'
            )
        else:
            msg.event_type = 'normal_check'
            msg.is_abnormal = False
            msg.description = (
                f'No hazard detected at waypoint '
                f'{request.waypoint_id} ({request.waypoint_name})'
            )

        return msg


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
