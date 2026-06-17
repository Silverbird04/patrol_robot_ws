import math

import cv2
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

from patrol_msgs.msg import CheckRequest
from patrol_msgs.msg import HazardEvent


class YoloHazardDetector(Node):
    def __init__(self):
        super().__init__('yolo_hazard_detector')

        self.bridge = CvBridge()

        self.latest_request = None
        self.pending_check = False
        self.frame_count = 0

        self.latest_scan = None
        self.front_min_distance = None

        self.last_event = None
        self.last_result_text = 'WAITING FOR CHECK REQUEST'
        self.last_result_color = (0, 255, 255)

        self.last_person_found = False
        self.last_person_boxes = []
        self.last_yolo_text = 'YOLO not run yet'

        # Demo-friendly obstacle detection setting
        self.obstacle_distance_threshold = 1.7
        self.front_angle_deg = 30.0

        # YOLO setting
        self.yolo_model = None
        self.yolo_available = False
        self.yolo_conf = 0.15
        self.yolo_imgsz = 320
        self.yolo_device = 'cpu'
        self.yolo_interval = 5

        # If True, person box center must be inside ROI.
        # For Gazebo actor demo, False is more stable because the actor may not be exactly centered.
        self.require_person_in_roi = False

        self.load_yolo_model()

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

        self.get_logger().info('YOLO hazard detector started.')
        self.get_logger().info('Subscribing to /camera/image_raw, /scan, and /check_request.')
        self.get_logger().info(
            f'Obstacle rule: front +/- {self.front_angle_deg:.1f} deg, '
            f'distance < {self.obstacle_distance_threshold:.2f} m'
        )

        if self.yolo_available:
            self.get_logger().info('YOLOv8 person detector is available.')
        else:
            self.get_logger().warn('YOLOv8 person detector is NOT available. Person detection will not trigger.')

    def load_yolo_model(self):
        try:
            from ultralytics import YOLO

            self.yolo_model = YOLO('yolov8n.pt')
            self.yolo_available = True

        except Exception as e:
            self.yolo_model = None
            self.yolo_available = False
            self.get_logger().warn(f'Failed to load YOLOv8 model: {e}')

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.front_min_distance = self.compute_front_min_distance(msg)

    def check_request_callback(self, msg):
        self.latest_request = msg
        self.pending_check = True

        self.last_result_text = 'CHECK REQUEST RECEIVED'
        self.last_result_color = (0, 255, 255)

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

        should_show_yolo = self.should_use_yolo_for_current_request()

        # Run YOLO periodically for display while in restricted area.
        if should_show_yolo and self.yolo_available:
            if self.frame_count % self.yolo_interval == 0:
                self.last_person_found, self.last_person_boxes = self.detect_person(frame, roi)
                self.last_yolo_text = 'FOUND' if self.last_person_found else 'not found'

        if self.pending_check and self.latest_request is not None:
            event_msg = self.perform_check(self.latest_request, frame, roi)
            self.event_pub.publish(event_msg)
            self.last_event = event_msg

            if event_msg.is_abnormal:
                self.last_result_text = self.make_abnormal_result_text(event_msg)
                self.last_result_color = (0, 0, 255)

                self.get_logger().warn(
                    f'Published hazard event: '
                    f'{event_msg.event_type}, '
                    f'waypoint={event_msg.waypoint_id}'
                )
            else:
                self.last_result_text = self.make_normal_result_text(event_msg)
                self.last_result_color = (0, 255, 0)

                self.get_logger().info(
                    f'Published normal event: '
                    f'{event_msg.event_type}, '
                    f'waypoint={event_msg.waypoint_id}'
                )

            self.pending_check = False

        self.draw_yolo_boxes(frame)
        self.draw_dashboard(frame)
        self.draw_roi(frame, roi)

        cv2.imshow('Patrol YOLO Hazard Detector', frame)
        cv2.waitKey(1)

    def should_use_yolo_for_current_request(self):
        if self.latest_request is None:
            return False

        return (
            self.latest_request.waypoint_type == 'restricted_area'
            and 'person' in self.latest_request.check_items
        )

    def get_roi(self, width, height):
        x1 = int(width * 0.30)
        y1 = int(height * 0.20)
        x2 = int(width * 0.70)
        y2 = int(height * 0.85)

        return x1, y1, x2, y2

    def detect_person(self, frame, roi):
        if not self.yolo_available or self.yolo_model is None:
            return False, []

        try:
            results = self.yolo_model.predict(
                source=frame,
                imgsz=self.yolo_imgsz,
                conf=self.yolo_conf,
                classes=[0],
                device=self.yolo_device,
                verbose=False
            )

            boxes = []
            person_found = False

            if not results:
                return False, []

            result = results[0]

            if result.boxes is None:
                return False, []

            for box in result.boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())

                x1, y1, x2, y2 = [
                    int(value) for value in xyxy
                ]

                if self.require_person_in_roi:
                    if not self.is_box_center_in_roi((x1, y1, x2, y2), roi):
                        continue

                boxes.append(
                    {
                        'xyxy': (x1, y1, x2, y2),
                        'conf': conf
                    }
                )

                person_found = True

            return person_found, boxes

        except Exception as e:
            self.get_logger().warn(f'YOLO inference failed: {e}')
            return False, []

    def is_box_center_in_roi(self, box, roi):
        x1, y1, x2, y2 = box
        rx1, ry1, rx2, ry2 = roi

        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0

        return rx1 <= cx <= rx2 and ry1 <= cy <= ry2

    def draw_yolo_boxes(self, frame):
        for detection in self.last_person_boxes:
            x1, y1, x2, y2 = detection['xyxy']
            conf = detection['conf']

            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                (255, 0, 0),
                2
            )

            cv2.putText(
                frame,
                f'person {conf:.2f}',
                (x1, max(20, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 0),
                2
            )

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
            (690, 260),
            (0, 0, 0),
            -1
        )

        cv2.rectangle(
            frame,
            (10, 10),
            (690, 260),
            (255, 255, 255),
            1
        )

        self.put_text(
            frame,
            'Patrol YOLO Hazard Detector',
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

        if self.yolo_available:
            yolo_status = f'YOLOv8 person: {self.last_yolo_text}'
        else:
            yolo_status = 'YOLOv8 person: unavailable'

        self.put_text(
            frame,
            yolo_status,
            panel_x,
            panel_y + line_h * 6,
            (255, 255, 255)
        )

        self.put_text(
            frame,
            f'Result: {self.last_result_text}',
            panel_x,
            panel_y + line_h * 7,
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

    def perform_check(self, request, frame, roi):
        msg = HazardEvent()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.waypoint_id = request.waypoint_id
        msg.waypoint_type = request.waypoint_type

        msg.x = request.x
        msg.y = request.y
        msg.yaw = request.yaw

        # 1. Restricted area person detection using YOLOv8
        if (
            request.waypoint_type == 'restricted_area'
            and 'person' in request.check_items
        ):
            if not self.yolo_available:
                msg.event_type = 'normal_check'
                msg.is_abnormal = False
                msg.description = (
                    f'YOLOv8 unavailable. Person detection was skipped at '
                    f'waypoint {request.waypoint_id} ({request.waypoint_name})'
                )
                return msg

            person_found, boxes = self.detect_person(frame, roi)

            self.last_person_found = person_found
            self.last_person_boxes = boxes
            self.last_yolo_text = 'FOUND' if person_found else 'not found'

            if person_found:
                msg.event_type = 'person_intrusion'
                msg.is_abnormal = True
                msg.description = (
                    f'YOLOv8 person detected at restricted area waypoint '
                    f'{request.waypoint_id} ({request.waypoint_name})'
                )
                return msg

            msg.event_type = 'normal_check'
            msg.is_abnormal = False
            msg.description = (
                f'No person detected by YOLOv8 at restricted area waypoint '
                f'{request.waypoint_id} ({request.waypoint_name})'
            )
            return msg

        # 2. Passage obstacle detection using front LiDAR
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

        # 3. Normal case
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
            return 'ABNORMAL - YOLO PERSON INTRUSION'

        if event_msg.event_type == 'obstacle_detected':
            return 'ABNORMAL - OBSTACLE'

        return f'ABNORMAL - {event_msg.event_type}'

    def make_normal_result_text(self, event_msg):
        return 'NORMAL - NO HAZARD'


def main(args=None):
    rclpy.init(args=args)

    node = YoloHazardDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('YOLO hazard detector stopped.')
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()