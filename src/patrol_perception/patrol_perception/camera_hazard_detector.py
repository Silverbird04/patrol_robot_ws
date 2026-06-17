import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan

from patrol_msgs.msg import CheckRequest, HazardEvent


class CameraHazardDetector(Node):
    def __init__(self):
        super().__init__('camera_hazard_detector')

        self.bridge = CvBridge()

        self.latest_request = None
        self.pending_check = False

        self.latest_scan = None
        self.front_min_distance = None

        self.last_result_text = 'WAITING FOR CHECK REQUEST'
        self.last_result_color = (0, 255, 255)

        self.last_person_found = False
        self.last_person_boxes = []
        self.last_person_pixels = 0

        self.obstacle_distance_threshold = 1.7
        self.front_angle_deg = 30.0

        self.frame_count = 0
        self.display_every_n_frames = 3

        # Camera-based demo person detection parameters
        # This is a lightweight color/ROI-based detector for the Gazebo actor.
        self.min_person_pixels = 30
        self.min_person_box_area = 20

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

        self.get_logger().info('Camera hazard detector started.')
        self.get_logger().info('Person detection: OpenCV color/ROI-based Gazebo actor detection.')
        self.get_logger().info('Obstacle detection: front LiDAR sector distance check.')

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
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        height, width, _ = frame.shape
        roi = self.get_roi(width, height)

        self.frame_count += 1

        # Always update camera-based person detection for visualization and stable checking.
        # The event decision still uses waypoint_type and check_items,
        # but the camera view always shows whether a person-like actor is visible.
        (
            self.last_person_found,
            self.last_person_boxes,
            self.last_person_pixels
        ) = self.detect_demo_person(frame, roi)

        if self.pending_check and self.latest_request is not None:
            event_msg = self.perform_check(self.latest_request, frame, roi)

            self.event_pub.publish(event_msg)

            if event_msg.is_abnormal:
                self.last_result_text = self.make_abnormal_result_text(event_msg)
                self.last_result_color = (0, 0, 255)

                self.get_logger().warn(
                    f'Published hazard event: '
                    f'{event_msg.event_type}, waypoint={event_msg.waypoint_id}'
                )
            else:
                self.last_result_text = 'NORMAL - NO HAZARD'
                self.last_result_color = (0, 255, 0)

                self.get_logger().info(
                    f'Published normal event: '
                    f'{event_msg.event_type}, waypoint={event_msg.waypoint_id}'
                )

            self.pending_check = False
        if self.frame_count % self.display_every_n_frames == 0:
            self.draw_roi(frame, roi)
            self.draw_person_boxes(frame)
            self.draw_compact_dashboard(frame)

            cv2.imshow('Patrol Camera Hazard Detector', frame)
            cv2.waitKey(1)

    def should_use_camera_person_detection(self):
        if self.latest_request is None:
            return False

        return (
            self.latest_request.waypoint_type == 'restricted_area'
            and 'person' in self.latest_request.check_items
        )

    def get_roi(self, width, height):
        # Wider ROI because Gazebo actor may not be perfectly centered
        x1 = int(width * 0.15)
        y1 = int(height * 0.05)
        x2 = int(width * 0.85)
        y2 = int(height * 0.95)

        return x1, y1, x2, y2

    def detect_demo_person(self, frame, roi):
        x1, y1, x2, y2 = roi
        roi_img = frame[y1:y2, x1:x2]

        hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)

        # The Gazebo actor often contains saturated green/blue colors.
        # Gray floor has low saturation, so it is filtered out.
        green_mask = cv2.inRange(
            hsv,
            np.array([30, 20, 20]),
            np.array([100, 255, 255])
        )

        blue_mask = cv2.inRange(
            hsv,
            np.array([90, 20, 10]),
            np.array([145, 255, 255])
        )

        dark_cloth_mask = cv2.inRange(
            hsv,
            np.array([20, 15, 10]),
            np.array([160, 255, 120])
        )

        person_mask = cv2.bitwise_or(green_mask, blue_mask)
        person_mask = cv2.bitwise_or(person_mask, dark_cloth_mask)

        kernel = np.ones((5, 5), np.uint8)
        person_mask = cv2.morphologyEx(person_mask, cv2.MORPH_OPEN, kernel)
        person_mask = cv2.morphologyEx(person_mask, cv2.MORPH_CLOSE, kernel)

        person_pixels = int(cv2.countNonZero(person_mask))

        contours, _ = cv2.findContours(
            person_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        boxes = []
        found = False

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < self.min_person_box_area:
                continue

            bx, by, bw, bh = cv2.boundingRect(contour)

            # Convert ROI-local coordinates to full-frame coordinates
            abs_box = (
                x1 + bx,
                y1 + by,
                x1 + bx + bw,
                y1 + by + bh
            )

            boxes.append(
                {
                    'xyxy': abs_box,
                    'area': area
                }
            )

            found = True

        if person_pixels < self.min_person_pixels:
            found = False
            boxes = []

        return found, boxes, person_pixels

    def perform_check(self, request, frame, roi):
        msg = HazardEvent()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.waypoint_id = request.waypoint_id
        msg.waypoint_type = request.waypoint_type

        msg.x = request.x
        msg.y = request.y
        msg.yaw = request.yaw

        # 1. Camera-based person detection for restricted area
        if (
            request.waypoint_type == 'restricted_area'
            and 'person' in request.check_items
        ):
            person_found, boxes, pixels = self.detect_demo_person(frame, roi)

            self.last_person_found = person_found
            self.last_person_boxes = boxes
            self.last_person_pixels = pixels

            if person_found:
                msg.event_type = 'person_intrusion'
                msg.is_abnormal = True
                msg.description = (
                    f'Camera-based person-like actor detected at restricted area '
                    f'waypoint {request.waypoint_id} ({request.waypoint_name}), '
                    f'color_pixels={pixels}'
                )
                return msg

            msg.event_type = 'normal_check'
            msg.is_abnormal = False
            msg.description = (
                f'No person-like actor detected by camera at restricted area '
                f'waypoint {request.waypoint_id} ({request.waypoint_name}), '
                f'color_pixels={pixels}'
            )
            return msg

        # 2. LiDAR-based obstacle detection for passage
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

    def is_front_obstacle_detected(self):
        if self.front_min_distance is None:
            return False

        return self.front_min_distance < self.obstacle_distance_threshold

    def draw_person_boxes(self, frame):
        for detection in self.last_person_boxes:
            x1, y1, x2, y2 = detection['xyxy']
            area = detection['area']

            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                (255, 0, 0),
                2
            )

            cv2.putText(
                frame,
                f'camera person area={area:.0f}',
                (x1, max(20, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
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
            (x1, max(20, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            self.last_result_color,
            2
        )
    
    def draw_compact_dashboard(self, frame):
        height, width, _ = frame.shape

        panel_height = 95

        overlay = frame.copy()
        cv2.rectangle(
            overlay,
            (0, 0),
            (width, panel_height),
            (0, 0, 0),
            -1
        )

        cv2.addWeighted(
            overlay,
            0.65,
            frame,
            0.35,
            0,
            frame
        )

        if self.latest_request is None:
            waypoint_text = 'Waypoint: none | Waiting for /check_request'
            mode_text = ''
        else:
            req = self.latest_request
            waypoint_text = (
                f'WP {req.waypoint_id}: {req.waypoint_name} | '
                f'{req.waypoint_type} | {list(req.check_items)}'
            )
            mode_text = (
                f'Pose x={req.x:.2f}, y={req.y:.2f}, yaw={req.yaw:.2f}'
            )

        person_status = 'FOUND' if self.last_person_found else 'not found'

        line1 = waypoint_text
        line2 = (
            f'LiDAR: {self.make_scan_text()} | '
            f'Camera person: {person_status} | '
            f'pixels={self.last_person_pixels}'
        )
        line3 = f'Result: {self.last_result_text}'

        cv2.putText(
            frame,
            line1,
            (15, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1
        )

        cv2.putText(
            frame,
            line2,
            (15, 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1
        )

        cv2.putText(
            frame,
            line3,
            (15, 85),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
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
            (730, 270),
            (0, 0, 0),
            -1
        )

        cv2.rectangle(
            frame,
            (10, 10),
            (730, 270),
            (255, 255, 255),
            1
        )

        self.put_text(
            frame,
            'Patrol Camera Hazard Detector',
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

        person_status = 'FOUND' if self.last_person_found else 'not found'

        self.put_text(
            frame,
            f'Camera person: {person_status} | color pixels={self.last_person_pixels}',
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

    def make_abnormal_result_text(self, event_msg):
        if event_msg.event_type == 'person_intrusion':
            return 'ABNORMAL - CAMERA PERSON INTRUSION'

        if event_msg.event_type == 'obstacle_detected':
            return 'ABNORMAL - OBSTACLE'

        return f'ABNORMAL - {event_msg.event_type}'


def main(args=None):
    rclpy.init(args=args)

    node = CameraHazardDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Camera hazard detector stopped.')
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()