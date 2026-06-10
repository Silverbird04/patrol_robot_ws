import json
import time

import cv2
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class DummyHazardDetector(Node):
    def __init__(self):
        super().__init__('dummy_hazard_detector')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.hazard_pub = self.create_publisher(
            String,
            '/hazard_event',
            10
        )

        self.frame_count = 0
        self.last_publish_time = 0.0

        self.get_logger().info('Dummy hazard detector started.')

    def image_callback(self, msg):
        self.frame_count += 1

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        height, width, _ = frame.shape

        # 화면 중앙 ROI
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

        # 임시 이상 감지 조건:
        # 100프레임마다 한 번 이상 상황 발생했다고 가정
        if self.frame_count % 100 == 0:
            self.publish_dummy_hazard()

            cv2.putText(
                frame,
                'DUMMY HAZARD DETECTED',
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

        cv2.imshow('Dummy Hazard Detector', frame)
        cv2.waitKey(1)

    def publish_dummy_hazard(self):
        now = time.time()

        # 너무 자주 publish하지 않도록 제한
        if now - self.last_publish_time < 2.0:
            return

        self.last_publish_time = now

        event = {
            'event_type': 'dummy_hazard',
            'waypoint_id': 0,
            'waypoint_type': 'test_area',
            'is_abnormal': True,
            'description': 'Dummy hazard detected in inspection ROI'
        }

        msg = String()
        msg.data = json.dumps(event)

        self.hazard_pub.publish(msg)
        self.get_logger().warn(f'Published hazard event: {msg.data}')


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