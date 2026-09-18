'''
[EN]
ROS 2 node that feeds Isaac Sim camera data to BookDetector.
ROS 2 node that feeds Isaac Sim camera data to TargetDetector.
[KR] 
Isaac Sim 카메라 데이터를 BookDetector로 전달하는 ROS 2 노드.
Isaac Sim 카메라 데이터를 TargetDetector로 전달하는 ROS 2 노드.
'''

import rclpy
import cv2
import math
import message_filters
import numpy as np
from ultralytics import YOLO

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from tf2_geometry_msgs import do_transform_point, do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener

from .book_detector import BookDetector
# from .target_detector import TargetDetector



class VisionManager(Node):
    def __init__(self):
        super().__init__('vision_manager')

        self.bridge = CvBridge()
        self.rgb_topic = self.declare_parameter(
            'rgb_topic', '/rgb').value
        self.depth_topic = self.declare_parameter(
            'depth_topic', '/depth').value
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/camera_info').value
        self.trigger_topic = self.declare_parameter(
            'trigger_topic', '/perception/detect_request').value
        self.wait_for_trigger = bool(self.declare_parameter(
            'wait_for_trigger', True).value)
        self.camera_frame = self.declare_parameter(
            'camera_frame', 'sim_camera').value
        self.target_frame = self.declare_parameter(
            'target_frame', 'arm_base_link').value
        self.depth_registered = bool(self.declare_parameter(
            'depth_registered', True).value)
        self.depth_scale = float(self.declare_parameter(
            'depth_scale', 0.0).value)
        self.sync_slop = float(self.declare_parameter('sync_slop', 0.1).value)
        self.book_topic = self.declare_parameter(
            'book_topic', '/perception/books').value
        self.book_pose_topic = self.declare_parameter(
            'book_pose_topic', '/perception/book_pose').value
        self.gripper_roll = float(self.declare_parameter(
            'gripper_roll', 0.0).value)
        self.gripper_pitch = float(self.declare_parameter(
            'gripper_pitch', 0.0).value)
        self.gripper_yaw_offset = float(self.declare_parameter(
            'gripper_yaw_offset', 0.0).value)
        # Shelf targeting is disabled until the shelf model and slot contract
        # are finalized.
        # self.target_slot_topic = self.declare_parameter(
        #     'target_slot_topic', '/perception/target_slot').value
        # self.slot_width = float(self.declare_parameter('slot_width', 0.3).value)
        # self.slot_height = float(self.declare_parameter('slot_height', 0.3).value)
        # self.insertion_depth = float(self.declare_parameter(
        #     'insertion_depth', 0.25).value)
        # self.pre_insert_offset = float(self.declare_parameter(
        #     'pre_insert_offset', 0.05).value)
        # self.target_confidence = float(self.declare_parameter(
        #     'target_confidence', 1.0).value)
        self.confidence_threshold = float(self.declare_parameter(
            'confidence_threshold', 0.75).value)

        # book_dataset에서 학습된 YOLO 모델(.pt) 파일 경로입니다.
        # 다른 모델을 사용할 때만 실행 시 -p model_path:=... 로 덮어쓰세요.
        self.model_path = self.declare_parameter(
            'model_path',
            '/home/rokey/book_dataset/runs/segment/runs/book/weights/best.pt',
        ).value
        # self.target_topic = self.declare_parameter(
        #     'target_topic', '/perception/empty_shelf_position').value
        # self.scan_radius = float(self.declare_parameter('scan_radius', 0.15).value)
        # self.scan_pixel_u = int(self.declare_parameter('scan_pixel_u', -1).value)
        # self.scan_pixel_v = int(self.declare_parameter('scan_pixel_v', -1).value)


        if not self.model_path:
            raise ValueError(
                'model_path parameter is required, for example '
                '-p model_path:=/path/to/book_best.pt')

        self.model = YOLO(self.model_path)
        self.book_detector = BookDetector()
        # self.target_detector = TargetDetector(self.scan_radius)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.book_pub = self.create_publisher(PointStamped, self.book_topic, 10)
        self.book_pose_pub = self.create_publisher(
            PoseStamped, self.book_pose_topic, 10)
        # self.target_pub = self.create_publisher(PointStamped, self.target_topic, 10)
        # self.target_slot_pub = self.create_publisher(
        #     TargetSlot, self.target_slot_topic, 10)
        self.trigger_sub = self.create_subscription(
            Bool, self.trigger_topic, self.trigger_callback, 10)
        self.pending_detection = not self.wait_for_trigger

        rgb_sub = message_filters.Subscriber(self, Image, self.rgb_topic)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, self.camera_info_topic)
        self.image_sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, camera_info_sub], 10, self.sync_slop)
        self.image_sync.registerCallback(self.rgb_callback)

        self.get_logger().info(
            f'Subscribed to rgb={self.rgb_topic}, depth={self.depth_topic}, '
            f'camera_info={self.camera_info_topic}, '
            f'target_frame={self.target_frame}, '
            f'trigger={self.trigger_topic}')

    def trigger_callback(self, msg):
        """Arm one synchronized image processing cycle on a true request."""
        if msg.data:
            self.pending_detection = True

    def rgb_callback(self, rgb_msg, depth_msg, camera_info_msg):
        if not self.pending_detection:
            return

        if not rgb_msg.header.frame_id or not depth_msg.header.frame_id:
            self.get_logger().warning('RGB or depth frame_id is empty')
            return
        if (rgb_msg.header.frame_id != self.camera_frame
                or depth_msg.header.frame_id != self.camera_frame
                or camera_info_msg.header.frame_id != self.camera_frame):
            self.get_logger().warning(
                f'Expected camera optical frame {self.camera_frame}, got '
                f'rgb={rgb_msg.header.frame_id}, '
                f'depth={depth_msg.header.frame_id}, '
                f'camera_info={camera_info_msg.header.frame_id}')
            return
        if (rgb_msg.header.frame_id != depth_msg.header.frame_id
                and not self.depth_registered):
            self.get_logger().warning(
                'RGB and depth frames differ; depth must be registered to RGB')
            return

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(
                rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough')
        except CvBridgeError as error:
            self.get_logger().error(f'Image conversion failed: {error}')
            return

        if rgb_image.shape[:2] != depth_image.shape[:2]:
            self.get_logger().warning('RGB and depth image sizes differ')
            return

        self.pending_detection = False
        depth_scale = self._get_depth_scale(depth_msg.encoding, depth_image)

        info = camera_info_msg
        fx, fy = info.k[0], info.k[4]
        cx, cy = info.k[2], info.k[5]

        if fx <= 0 or fy <= 0:
            self.get_logger().warning('Invalid camera intrinsics in CameraInfo')
            return

        try:
            detections = self._detect_books(rgb_image)
            detected_targets = self.book_detector.process(
                detections,
                depth_image,
                fx,
                fy,
                cx,
                cy,
                depth_scale,
            )
        except (IndexError, ValueError) as error:
            self.get_logger().error(f'Book detection failed: {error}')
            return

        for target in detected_targets:
            book_point = self._transform_xyz(
                target['xyz'], rgb_msg.header.frame_id, rgb_msg.header.stamp)
            if book_point is None:
                continue
            book_msg = PointStamped()
            book_msg.header = book_point.header
            book_msg.point = book_point.point
            self.book_pub.publish(book_msg)
            book_pose = self._make_book_pose(
                target['xyz'],
                rgb_msg.header.frame_id,
                rgb_msg.header.stamp,
                target['image_angle'],
            )
            if book_pose is not None:
                self.book_pose_pub.publish(book_pose)
            self.get_logger().info(
                f"Book detected: xyz=({book_point.point.x:.3f}, "
                f"{book_point.point.y:.3f}, {book_point.point.z:.3f}), "
                f"center={target['center']}, "
                f"yaw={target['image_angle']:.3f} rad")

        # Shelf targeting is disabled for the book-only validation stage.
        # scan_xyz = self._scan_position(
        #     depth_image, fx, fy, cx, cy, depth_scale)
        # empty_position = self.target_detector.process(detected_targets, scan_xyz)
        # if empty_position is None:
        #     self.get_logger().info('No placeable empty shelf position found')
        #     return
        #
        # target_point = self._transform_xyz(
        #     empty_position['xyz'], rgb_msg.header.frame_id, rgb_msg.header.stamp)
        # if target_point is None:
        #     return
        #
        # self.target_pub.publish(target_point)
        # target_slot = TargetSlot()
        # target_slot.header = target_point.header
        # target_slot.pose.position.x = target_point.point.x
        # target_slot.pose.position.y = target_point.point.y
        # target_slot.pose.position.z = target_point.point.z
        # target_slot.pose.orientation.w = 1.0
        # target_slot.available_width = self.slot_width
        # target_slot.available_height = self.slot_height
        # target_slot.insertion_depth = self.insertion_depth
        # target_slot.pre_insert_offset = self.pre_insert_offset
        # target_slot.confidence = self.target_confidence
        # self.target_slot_pub.publish(target_slot)

    def _transform_xyz(self, xyz, source_frame, stamp):
        point = PointStamped()
        point.header.frame_id = source_frame
        point.header.stamp = stamp
        point.point.x, point.point.y, point.point.z = xyz
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time.from_msg(stamp),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            return do_transform_point(point, transform)
        except TransformException as error:
            self.get_logger().warning(
                f'Could not transform {source_frame} to '
                f'{self.target_frame}: {error}')
            return None

    def _make_book_pose(self, xyz, source_frame, stamp, image_angle):
        yaw = image_angle + self.gripper_yaw_offset
        pose = PoseStamped()
        pose.header.frame_id = source_frame
        pose.header.stamp = stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
        quaternion = self._quaternion_from_rpy(
            self.gripper_roll, self.gripper_pitch, yaw)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time.from_msg(stamp),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            return do_transform_pose(pose, transform)
        except TransformException as error:
            self.get_logger().warning(
                f'Could not transform book pose from {source_frame} to '
                f'{self.target_frame}: {error}')
            return None

    def _quaternion_from_rpy(self, roll, pitch, yaw):
        half_roll = roll * 0.5
        half_pitch = pitch * 0.5
        half_yaw = yaw * 0.5
        cr, sr = math.cos(half_roll), math.sin(half_roll)
        cp, sp = math.cos(half_pitch), math.sin(half_pitch)
        cy, sy = math.cos(half_yaw), math.sin(half_yaw)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _get_depth_scale(self, encoding, depth_image):
        if self.depth_scale > 0:
            return self.depth_scale
        if depth_image.dtype.name == 'uint16' or encoding == '16UC1':
            return 0.001
        return 1.0

    # def _scan_position(self, depth_image, fx, fy, cx, cy, depth_scale):
    #     height, width = depth_image.shape[:2]
    #     u = self.scan_pixel_u if self.scan_pixel_u >= 0 else width // 2
    #     v = self.scan_pixel_v if self.scan_pixel_v >= 0 else height // 2
    #     if not (0 <= u < width and 0 <= v < height):
    #         self.get_logger().warning('Scan pixel is outside the depth image')
    #         return None
    #
    #     depth = float(depth_image[v, u]) * depth_scale
    #     if not math.isfinite(depth) or depth <= 0:
    #         return None
    #
    #     return (
    #         (u - cx) * depth / fx,
    #         (v - cy) * depth / fy,
    #         depth,
    #     )

    def _detect_books(self, rgb_image):
        detections = []
        results = self.model(rgb_image, verbose=False)
        for result in results:
            masks = result.masks.data if result.masks is not None else None
            for index, box in enumerate(result.boxes):
                confidence = float(box.conf[0])
                if confidence < self.confidence_threshold:
                    continue
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                if class_name != 'book':
                    continue
                mask = None
                if masks is not None and index < len(masks):
                    mask = masks[index].cpu().numpy() > 0.5
                    if mask.shape != rgb_image.shape[:2]:
                        mask = cv2.resize(
                            mask.astype(np.uint8),
                            (rgb_image.shape[1], rgb_image.shape[0]),
                            interpolation=cv2.INTER_NEAREST,
                        ).astype(bool)
                detections.append({
                    'box': tuple(map(int, box.xyxy[0])),
                    'mask': mask,
                    'confidence': confidence,
                })
        return detections



def main(args=None):

    rclpy.init(args=args)

    node = VisionManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().warn("강제 종료")

    finally:

        cv2.destroyAllWindows()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()