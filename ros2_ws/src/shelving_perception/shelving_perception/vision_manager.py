'''
[EN]
ROS 2 node that feeds Isaac Sim camera data to BookDetector.
ROS 2 node that feeds Isaac Sim camera data to TargetDetector.
[KR] 
Isaac Sim 카메라 데이터를 BookDetector로 전달하는 ROS 2 노드.
Isaac Sim 카메라 데이터를 TargetDetector로 전달하는 ROS 2 노드.
'''

# ROS 2 Python 클라이언트 라이브러리입니다.
import rclpy
# 종료 시 OpenCV 창을 정리하기 위해 가져옵니다.
import cv2
# roll/pitch/yaw를 사원수로 바꿀 때 사용할 삼각함수 모듈입니다.
import math
# RGB·Depth·CameraInfo를 시간 기준으로 묶어주는 ROS 메시지 필터입니다.
import message_filters
# 배열 크기와 수치 계산에 사용하는 NumPy입니다.
import numpy as np
# 여러 ROS 콜백이 동시에 실행될 때 공유 상태를 보호하는 모듈입니다.
import threading
# action timeout을 실제 경과 시간으로 측정하는 모듈입니다.
import time
# 학습된 YOLO 모델을 불러오고 추론하는 라이브러리입니다.
from ultralytics import YOLO

# ROS 2 action 서버를 구성하고 goal 수락·취소 응답을 만들 때 사용합니다.
from rclpy.action import ActionServer, CancelResponse, GoalResponse
# action 실행 콜백과 센서 콜백을 함께 처리할 수 있는 콜백 그룹입니다.
from rclpy.callback_groups import ReentrantCallbackGroup
# task_manager_node와 주고받는 검출 action 인터페이스입니다.
from shelving_interfaces.action import DetectTargetSlot
# 향후 선반 슬롯 메시지에 사용할 타입입니다.
from shelving_interfaces.msg import TargetSlot
# ROS 2 노드의 기본 클래스입니다.
from rclpy.node import Node
# RGB/Depth 영상과 카메라 내부 파라미터 메시지입니다.
from sensor_msgs.msg import Image, CameraInfo
# 책 위치와 책 pose를 발행할 메시지입니다.
from geometry_msgs.msg import PointStamped, PoseStamped
# 수동 검출 요청 토픽의 Boolean 메시지입니다.
from std_msgs.msg import Bool
# ROS Image 메시지를 OpenCV 영상으로 바꾸는 브리지입니다.
from cv_bridge import CvBridge, CvBridgeError
# 카메라 좌표를 로봇 좌표로 변환하는 함수입니다.
from tf2_geometry_msgs import (
    do_transform_point,
    do_transform_pose_stamped,
)
# TF를 조회하고 TF 조회 실패를 처리하기 위한 클래스입니다.
from tf2_ros import Buffer, TransformException, TransformListener

# 깊이값을 이용해 책의 3차원 위치를 계산하는 보조 클래스입니다.
from .book_detector import BookDetector
# from .target_detector import TargetDetector



class VisionManager(Node):
    """카메라 영상을 받아 책을 검출하고 ROS 결과로 변환하는 노드입니다."""

    def __init__(self):
        # 노드 이름을 vision_manager로 등록합니다.
        super().__init__('vision_manager')

        # ROS Image와 OpenCV 이미지 사이의 변환 도구를 생성합니다.
        self.bridge = CvBridge()
        # RGB 영상이 들어오는 토픽 이름을 파라미터에서 읽습니다.
        self.rgb_topic = self.declare_parameter(
            'rgb_topic', '/rgb').value
        # 깊이 영상이 들어오는 토픽 이름을 파라미터에서 읽습니다.
        self.depth_topic = self.declare_parameter(
            'depth_topic', '/depth').value
        # 카메라 내부 파라미터가 들어오는 토픽 이름을 읽습니다.
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/camera_info').value
        # 수동 검출 요청을 받을 Boolean 토픽 이름을 읽습니다.
        self.trigger_topic = self.declare_parameter(
            'trigger_topic', '/perception/detect_request').value
        # task_manager와 통신할 action 이름을 읽습니다.
        self.perception_action = self.declare_parameter(
            'perception_action', '/detect_target_slot').value
        # True이면 요청을 받은 뒤에만 한 프레임을 처리합니다.
        self.wait_for_trigger = bool(self.declare_parameter(
            'wait_for_trigger', True).value)
        # RGB·Depth·CameraInfo가 촬영된 카메라 frame 이름입니다.
        self.camera_frame = self.declare_parameter(
            'camera_frame', 'sim_camera').value
        # 검출 좌표를 변환할 최종 로봇 frame 이름입니다.
        self.target_frame = self.declare_parameter(
            'target_frame', 'arm_base_link').value
        # RGB와 Depth가 서로 정렬되어 있는지 나타내는 설정입니다.
        self.depth_registered = bool(self.declare_parameter(
            'depth_registered', True).value)
        # 깊이 원시값을 미터로 변환할 배율입니다. 0이면 encoding으로 자동 결정합니다.
        self.depth_scale = float(self.declare_parameter(
            'depth_scale', 0.0).value)
        # 세 센서 메시지를 같은 프레임으로 인정할 최대 시간 차이입니다.
        self.sync_slop = float(self.declare_parameter('sync_slop', 0.1).value)
        # 책의 3차원 점을 발행할 토픽입니다.
        self.book_topic = self.declare_parameter(
            'book_topic', '/perception/books').value
        # 책의 위치·자세를 발행할 토픽입니다.
        self.book_pose_topic = self.declare_parameter(
            'book_pose_topic', '/perception/book_pose').value
        # 결과 pose에 적용할 gripper roll 보정값입니다.
        self.gripper_roll = float(self.declare_parameter(
            'gripper_roll', 0.0).value)
        # 결과 pose에 적용할 gripper pitch 보정값입니다.
        self.gripper_pitch = float(self.declare_parameter(
            'gripper_pitch', 0.0).value)
        # 영상에서 계산한 yaw에 더할 gripper yaw 보정값입니다.
        self.gripper_yaw_offset = float(self.declare_parameter(
            'gripper_yaw_offset', 0.0).value)
        # action 결과 TargetSlot에 넣을 삽입 깊이입니다.
        self.insertion_depth = float(self.declare_parameter(
            'insertion_depth', 0.25).value)
        # 삽입 전 대기 위치의 오프셋입니다.
        self.pre_insert_offset = float(self.declare_parameter(
            'pre_insert_offset', 0.05).value)
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
        # YOLO 검출 결과를 책으로 인정할 최소 confidence입니다.
        self.confidence_threshold = float(self.declare_parameter(
            'confidence_threshold', 0.75).value)

        # 도서관 데이터로 학습된 YOLO 모델(.pt) 파일 경로입니다.
        # 다른 모델을 사용할 때만 실행 시 -p model_path:=... 로 덮어쓰세요.
        # 실행할 학습된 YOLO weight 파일 경로입니다.
        self.model_path = self.declare_parameter(
            'model_path',
            '/home/rokey/도서관/260918_bag_vision/model/book_tray_best.pt',
        ).value
        # self.target_topic = self.declare_parameter(
        #     'target_topic', '/perception/empty_shelf_position').value
        # self.scan_radius = float(self.declare_parameter('scan_radius', 0.15).value)
        # self.scan_pixel_u = int(self.declare_parameter('scan_pixel_u', -1).value)
        # self.scan_pixel_v = int(self.declare_parameter('scan_pixel_v', -1).value)


        # 모델 경로가 비어 있으면 초기화할 수 없으므로 즉시 오류를 냅니다.
        if not self.model_path:
            raise ValueError(
                'model_path parameter is required, for example '
                '-p model_path:=/path/to/book_best.pt')

        # 학습된 YOLO 모델을 메모리에 로드합니다.
        self.model = YOLO(self.model_path)
        # 검출 상자와 깊이로 3차원 좌표를 계산하는 객체를 생성합니다.
        self.book_detector = BookDetector()
        # self.target_detector = TargetDetector(self.scan_radius)
        # TF 변환을 조회할 버퍼와 listener를 생성합니다.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # action callback이 센서 callback과 동시에 실행될 수 있도록 그룹을 만듭니다.
        self._action_group = ReentrantCallbackGroup()
        # action 상태를 여러 스레드에서 안전하게 접근하기 위한 lock입니다.
        self._action_lock = threading.Lock()
        # 센서 처리 완료를 action 실행 콜백에 알려주는 이벤트입니다.
        self._action_event = threading.Event()
        # 현재 실행 중인 action goal을 저장합니다.
        self._active_goal = None
        # 센서 처리 결과를 action 콜백에 전달하기 위한 변수입니다.
        self._action_result = None
        # task_manager가 보내는 DetectTargetSlot goal을 받는 action 서버입니다.
        self._action_server = ActionServer(
            self,
            DetectTargetSlot,
            self.perception_action,
            execute_callback=self._execute_detection,
            goal_callback=self._accept_detection_goal,
            cancel_callback=self._cancel_detection_goal,
            callback_group=self._action_group,
        )
        # 책마다 변환된 3차원 위치를 PointStamped로 발행합니다.
        self.book_pub = self.create_publisher(PointStamped, self.book_topic, 10)
        # 책마다 변환된 pose를 PoseStamped로 발행합니다.
        self.book_pose_pub = self.create_publisher(
            PoseStamped, self.book_pose_topic, 10)
        # self.target_pub = self.create_publisher(PointStamped, self.target_topic, 10)
        # self.target_slot_pub = self.create_publisher(
        #     TargetSlot, self.target_slot_topic, 10)
        # 수동 테스트용 Boolean 검출 요청 subscriber를 생성합니다.
        self.trigger_sub = self.create_subscription(
            Bool, self.trigger_topic, self.trigger_callback, 10)
        # trigger를 기다리는 설정이면 False, 아니면 즉시 처리 가능하게 합니다.
        self.pending_detection = not self.wait_for_trigger

        # RGB 토픽을 message_filters subscriber로 연결합니다.
        rgb_sub = message_filters.Subscriber(self, Image, self.rgb_topic)
        # Depth 토픽을 message_filters subscriber로 연결합니다.
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        # CameraInfo 토픽을 message_filters subscriber로 연결합니다.
        camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, self.camera_info_topic)
        # 세 메시지의 timestamp가 가까운 것끼리 묶는 동기화기를 만듭니다.
        self.image_sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, camera_info_sub], 10, self.sync_slop)
        # 동기화된 세 메시지가 들어오면 rgb_callback을 실행합니다.
        self.image_sync.registerCallback(self.rgb_callback)

        # 현재 연결된 센서·action 설정을 로그로 출력합니다.
        self.get_logger().info(
            f'Subscribed to rgb={self.rgb_topic}, depth={self.depth_topic}, '
            f'camera_info={self.camera_info_topic}, '
            f'target_frame={self.target_frame}, '
            f'trigger={self.trigger_topic}, '
            f'action={self.perception_action}')

    def _accept_detection_goal(self, goal_request):
        """동시에 하나의 검출 action만 실행하도록 goal을 수락합니다."""
        # 현재 다른 goal이 실행 중인지 lock으로 안전하게 확인합니다.
        with self._action_lock:
            # 이미 처리 중인 goal이 있으면 새 goal을 거절합니다.
            if self._active_goal is not None:
                return GoalResponse.REJECT
        # 현재 처리 중인 goal이 없으므로 새 goal을 수락합니다.
        return GoalResponse.ACCEPT

    def _cancel_detection_goal(self, goal_handle):
        """진행 중인 검출 action의 취소 요청을 수락합니다."""
        # 실제 정리는 execute callback의 취소 검사에서 처리합니다.
        return CancelResponse.ACCEPT

    def _execute_detection(self, goal_handle):
        """action 요청 후 들어오는 다음 유효한 카메라 프레임을 처리합니다."""
        # action 상태를 센서 callback과 공유하므로 lock을 사용합니다.
        with self._action_lock:
            # 현재 활성 goal을 기록합니다.
            self._active_goal = goal_handle
            # 이전 action 결과를 비웁니다.
            self._action_result = None
            # 이전 처리 완료 이벤트를 초기화합니다.
            self._action_event.clear()
            # 다음 동기화 프레임에서 검출하도록 활성화합니다.
            self.pending_detection = True

        # action 클라이언트에게 촬영 단계가 시작되었음을 알립니다.
        self._publish_action_feedback(goal_handle, 'CAPTURING', 0, 0.0)
        # 카메라 프레임을 기다릴 최대 시간(초)입니다.
        timeout = 10.0
        # timeout 계산용 실제 시작 시각을 기록합니다.
        started = time.monotonic()
        # 센서 callback이 결과를 만들 때까지 짧게 반복 대기합니다.
        while not self._action_event.wait(0.1):
            # action 클라이언트가 취소했는지 확인합니다.
            if goal_handle.is_cancel_requested:
                # 활성 goal과 대기 상태를 정리합니다.
                with self._action_lock:
                    self._active_goal = None
                    self.pending_detection = False
                # action을 canceled 상태로 종료합니다.
                goal_handle.canceled()
                # 취소 결과 메시지를 생성합니다.
                result = DetectTargetSlot.Result()
                result.error_code = 3002
                result.message = 'Perception request was canceled.'
                return result
            # 현재까지 경과한 실제 시간을 계산합니다.
            now = time.monotonic()
            # 10초 안에 프레임이 오지 않으면 action을 실패시킵니다.
            if now - started > timeout:
                # timeout 상태를 정리합니다.
                with self._action_lock:
                    self._active_goal = None
                    self.pending_detection = False
                # action을 aborted 상태로 종료합니다.
                goal_handle.abort()
                # timeout 결과 메시지를 생성합니다.
                result = DetectTargetSlot.Result()
                result.error_code = 3003
                result.message = 'Timed out waiting for a valid camera frame.'
                return result

        # 센서 callback이 저장한 결과를 가져오고 active goal을 해제합니다.
        with self._action_lock:
            result = self._action_result
            self._active_goal = None

        # 결과가 비어 있으면 내부 오류로 action을 종료합니다.
        if result is None:
            goal_handle.abort()
            result = DetectTargetSlot.Result()
            result.error_code = 3004
            result.message = 'Perception returned no result.'
            return result

        # 검출 성공 여부에 따라 action 상태를 확정합니다.
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        # task_manager로 최종 결과를 반환합니다.
        return result

    def _publish_action_feedback(self, goal_handle, phase, count, confidence):
        # action feedback 메시지를 생성합니다.
        feedback = DetectTargetSlot.Feedback()
        # 현재 처리 단계를 설정합니다.
        feedback.phase = phase
        # 현재 검출 후보 개수를 설정합니다.
        feedback.candidate_count = count
        # 가장 높은 confidence를 설정합니다.
        feedback.best_confidence = confidence
        # action 클라이언트에 feedback을 전송합니다.
        goal_handle.publish_feedback(feedback)

    def trigger_callback(self, msg):
        """True 토픽을 받으면 한 번의 동기화 영상 처리를 예약합니다."""
        # True 요청일 때만 다음 센서 프레임을 처리하도록 활성화합니다.
        if msg.data:
            self.pending_detection = True

    def rgb_callback(self, rgb_msg, depth_msg, camera_info_msg):
        # action 또는 수동 trigger가 없으면 현재 프레임을 처리하지 않습니다.
        if not self.pending_detection:
            return

        # RGB와 Depth 메시지에 frame_id가 있는지 확인합니다.
        if not rgb_msg.header.frame_id or not depth_msg.header.frame_id:
            self.get_logger().warning('RGB or depth frame_id is empty')
            return
        # 세 메시지가 예상한 카메라 frame에서 왔는지 확인합니다.
        if (rgb_msg.header.frame_id != self.camera_frame
                or depth_msg.header.frame_id != self.camera_frame
                or camera_info_msg.header.frame_id != self.camera_frame):
            self.get_logger().warning(
                f'Expected camera optical frame {self.camera_frame}, got '
                f'rgb={rgb_msg.header.frame_id}, '
                f'depth={depth_msg.header.frame_id}, '
                f'camera_info={camera_info_msg.header.frame_id}')
            return
        # RGB와 Depth frame이 다르면 registered depth 설정을 확인합니다.
        if (rgb_msg.header.frame_id != depth_msg.header.frame_id
                and not self.depth_registered):
            self.get_logger().warning(
                'RGB and depth frames differ; depth must be registered to RGB')
            return

        try:
            # ROS RGB 메시지를 OpenCV BGR 이미지로 변환합니다.
            rgb_image = self.bridge.imgmsg_to_cv2(
                rgb_msg, desired_encoding='bgr8')
            # Depth 메시지는 원래 숫자 encoding을 유지한 채 변환합니다.
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough')
        except CvBridgeError as error:
            self.get_logger().error(f'Image conversion failed: {error}')
            return

        # RGB와 Depth의 가로·세로 크기가 같은지 확인합니다.
        if rgb_image.shape[:2] != depth_image.shape[:2]:
            self.get_logger().warning('RGB and depth image sizes differ')
            return

        # 이번 요청은 현재 동기화 프레임으로 처리했으므로 다시 대기 상태로 바꿉니다.
        self.pending_detection = False
        # Depth encoding과 자료형을 보고 미터 변환 배율을 결정합니다.
        depth_scale = self._get_depth_scale(depth_msg.encoding, depth_image)

        # CameraInfo에서 카메라 내부 파라미터를 가져옵니다.
        info = camera_info_msg
        # x/y 초점거리와 주점 좌표를 추출합니다.
        fx, fy = info.k[0], info.k[4]
        cx, cy = info.k[2], info.k[5]

        # 초점거리가 0 이하이면 3D 투영을 수행할 수 없습니다.
        if fx <= 0 or fy <= 0:
            self.get_logger().warning('Invalid camera intrinsics in CameraInfo')
            return

        try:
            # RGB 이미지에서 YOLO 책 검출 결과를 계산합니다.
            detections = self._detect_books(rgb_image)
            # 검출 상자와 Depth로 각 책의 카메라 기준 3D 위치를 계산합니다.
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
            # 모델 출력 또는 깊이 계산 오류를 action 실패 결과로 전달합니다.
            self.get_logger().error(f'Book detection failed: {error}')
            self._finish_action_with_failure(
                3003, f'Book detection failed: {error}')
            return

        # 현재 action goal을 안전하게 읽습니다.
        with self._action_lock:
            active_goal = self._active_goal
        # action 요청으로 처리 중이면 검출 단계 feedback을 전송합니다.
        if active_goal is not None:
            self._publish_action_feedback(
                active_goal,
                'DETECTING_SHELF',
                len(detected_targets),
                max(
                    (target['confidence'] for target in detected_targets),
                    default=0.0,
                ),
            )

        # 검출된 모든 책을 하나씩 로봇 좌표로 변환하고 발행합니다.
        for target in detected_targets:
            # 책의 카메라 좌표를 target_frame 기준 점으로 변환합니다.
            book_point = self._transform_xyz(
                target['xyz'], rgb_msg.header.frame_id, rgb_msg.header.stamp)
            # TF 변환에 실패한 책은 결과 발행을 건너뜁니다.
            if book_point is None:
                continue
            # 변환된 점을 담을 ROS 메시지를 생성합니다.
            book_msg = PointStamped()
            # TF 변환 결과의 frame과 timestamp를 복사합니다.
            book_msg.header = book_point.header
            # TF 변환 결과의 x/y/z 위치를 복사합니다.
            book_msg.point = book_point.point
            # 책의 3D 점을 외부 노드로 발행합니다.
            self.book_pub.publish(book_msg)
            # 책의 위치와 영상상의 yaw를 pose로 변환합니다.
            book_pose = self._make_book_pose(
                target['xyz'],
                rgb_msg.header.frame_id,
                rgb_msg.header.stamp,
                target['image_angle'],
            )
            # pose 변환에 성공한 경우에만 pose를 발행합니다.
            if book_pose is not None:
                self.book_pose_pub.publish(book_pose)
            # 사람이 확인할 수 있도록 검출 위치와 각도를 로그로 출력합니다.
            self.get_logger().info(
                f"Book detected: xyz=({book_point.point.x:.3f}, "
                f"{book_point.point.y:.3f}, {book_point.point.z:.3f}), "
                f"center={target['center']}, "
                f"yaw={target['image_angle']:.3f} rad")

        # action 요청이었다면 가장 신뢰도 높은 책을 action 결과로 만듭니다.
        if active_goal is not None:
            self._complete_action_from_books(
                active_goal,
                detected_targets,
                rgb_msg.header.frame_id,
                rgb_msg.header.stamp,
            )

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
        """카메라 기준 점을 target_frame 기준 점으로 변환합니다."""
        # 변환할 점 메시지를 생성합니다.
        point = PointStamped()
        # 점의 원래 frame을 설정합니다.
        point.header.frame_id = source_frame
        # 센서 timestamp를 유지해야 같은 시점의 TF를 조회할 수 있습니다.
        point.header.stamp = stamp
        # 계산된 카메라 기준 xyz를 메시지에 넣습니다.
        point.point.x, point.point.y, point.point.z = xyz
        try:
            # target_frame에서 source_frame으로 가는 TF를 해당 시각에 조회합니다.
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time.from_msg(stamp),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            # 조회한 TF를 점에 적용해 target_frame 기준 점을 반환합니다.
            return do_transform_point(point, transform)
        except TransformException as error:
            # TF가 없거나 timestamp가 오래되면 변환 실패를 로그로 남깁니다.
            self.get_logger().warning(
                f'Could not transform {source_frame} to '
                f'{self.target_frame}: {error}')
            return None

    def _make_book_pose(self, xyz, source_frame, stamp, image_angle):
        """책 위치와 영상 각도로 target_frame 기준 PoseStamped를 만듭니다."""
        # 영상에서 구한 책 방향에 gripper yaw 보정값을 더합니다.
        yaw = image_angle + self.gripper_yaw_offset
        # 책 pose 메시지를 생성합니다.
        pose = PoseStamped()
        # pose의 원래 frame을 카메라 frame으로 설정합니다.
        pose.header.frame_id = source_frame
        # 센서 timestamp를 pose에 기록합니다.
        pose.header.stamp = stamp
        # 카메라 기준 책 위치를 pose에 기록합니다.
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
        # roll/pitch/yaw를 ROS quaternion으로 변환합니다.
        quaternion = self._quaternion_from_rpy(
            self.gripper_roll, self.gripper_pitch, yaw)
        # 계산한 quaternion의 각 성분을 pose에 기록합니다.
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        try:
            # pose timestamp에 맞는 카메라→로봇 TF를 조회합니다.
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time.from_msg(stamp),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
            # Jazzy API에 맞는 PoseStamped용 TF 변환 함수를 호출합니다.
            return do_transform_pose_stamped(pose, transform)
        except TransformException as error:
            # TF 조회 실패 시 pose를 만들지 않고 None을 반환합니다.
            self.get_logger().warning(
                f'Could not transform book pose from {source_frame} to '
                f'{self.target_frame}: {error}')
            return None

    def _complete_action_from_books(
        self,
        goal_handle,
        detected_targets,
        source_frame,
        stamp,
    ):
        """가장 신뢰도 높은 책을 임시 TargetSlot action 결과로 반환합니다."""
        # action 결과 메시지를 생성합니다.
        result = DetectTargetSlot.Result()
        # 이번 프레임에서 찾은 후보 수를 기록합니다.
        result.candidate_count = len(detected_targets)
        # 책 후보가 하나도 없으면 실패 결과를 저장합니다.
        if not detected_targets:
            result.error_code = 3003
            result.message = 'No book candidate was detected.'
            self._set_action_result(result)
            return

        # confidence가 가장 높은 책을 action 대표 후보로 선택합니다.
        target = max(
            detected_targets,
            key=lambda candidate: candidate['confidence'],
        )
        # 대표 후보의 위치를 target_frame으로 변환합니다.
        point = self._transform_xyz(target['xyz'], source_frame, stamp)
        # 변환에 실패하면 action 실패 결과를 저장합니다.
        if point is None:
            result.error_code = 3004
            result.message = 'Could not transform detected book coordinates.'
            self._set_action_result(result)
            return

        # pose 계산 단계와 대표 confidence를 feedback으로 보냅니다.
        self._publish_action_feedback(
            goal_handle, 'CALCULATING_POSE', len(detected_targets),
            target['confidence'])
        # 결과 TargetSlot 객체에 대표 책의 위치를 기록합니다.
        slot = result.target_slot
        slot.header = point.header
        slot.pose.position.x = point.point.x
        slot.pose.position.y = point.point.y
        slot.pose.position.z = point.point.z
        # 대표 책의 영상 각도와 gripper 보정값으로 방향을 계산합니다.
        quaternion = self._quaternion_from_rpy(
            self.gripper_roll,
            self.gripper_pitch,
            target['image_angle'] + self.gripper_yaw_offset,
        )
        slot.pose.orientation.x = quaternion[0]
        slot.pose.orientation.y = quaternion[1]
        slot.pose.orientation.z = quaternion[2]
        slot.pose.orientation.w = quaternion[3]

        # action goal에서 책 크기와 안전 여유를 읽습니다.
        goal = goal_handle.request
        # 책을 잡을 때 필요한 허용 폭을 계산합니다.
        slot.available_width = max(
            0.0, float(goal.book_thickness + 2.0 * goal.safety_margin))
        # 책 높이와 안전 여유를 이용해 허용 높이를 계산합니다.
        slot.available_height = max(
            0.0, float(goal.book_height + goal.safety_margin))
        # 삽입 관련 파라미터를 결과 슬롯에 기록합니다.
        slot.insertion_depth = self.insertion_depth
        slot.pre_insert_offset = self.pre_insert_offset
        # 대표 검출의 confidence를 결과에 기록합니다.
        slot.confidence = target['confidence']
        # 현재 구현에서는 대표 책 후보를 성공 결과로 반환합니다.
        result.success = True
        result.error_code = 0
        result.message = 'Book candidate converted to TargetSlot.'
        # action의 마지막 변환 단계 feedback을 보냅니다.
        self._publish_action_feedback(
            goal_handle, 'TRANSFORMING_FRAME', len(detected_targets),
            slot.confidence)
        # execute callback이 결과를 가져갈 수 있도록 저장하고 이벤트를 깨웁니다.
        self._set_action_result(result)

    def _finish_action_with_failure(self, error_code, message):
        # 실패 상태를 담은 action 결과 메시지를 생성합니다.
        result = DetectTargetSlot.Result()
        # 실패 코드와 설명을 기록합니다.
        result.error_code = error_code
        result.message = message
        # 실행 중인 action에 실패 결과를 전달합니다.
        self._set_action_result(result)

    def _set_action_result(self, result):
        # action 상태를 센서 callback과 공유하므로 lock을 획득합니다.
        with self._action_lock:
            # 실행 중인 goal이 있을 때만 결과를 저장합니다.
            if self._active_goal is not None:
                self._action_result = result
                # execute callback의 대기 루프를 깨워 결과 처리를 진행합니다.
                self._action_event.set()

    def _quaternion_from_rpy(self, roll, pitch, yaw):
        # 회전각의 절반은 quaternion 변환 공식에 사용됩니다.
        half_roll = roll * 0.5
        half_pitch = pitch * 0.5
        half_yaw = yaw * 0.5
        # roll의 cos/sin 값을 계산합니다.
        cr, sr = math.cos(half_roll), math.sin(half_roll)
        # pitch의 cos/sin 값을 계산합니다.
        cp, sp = math.cos(half_pitch), math.sin(half_pitch)
        # yaw의 cos/sin 값을 계산합니다.
        cy, sy = math.cos(half_yaw), math.sin(half_yaw)
        # ROS가 사용하는 quaternion 순서 x, y, z, w로 반환합니다.
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _get_depth_scale(self, encoding, depth_image):
        # 양수 배율을 직접 지정했다면 자동 판정보다 우선합니다.
        if self.depth_scale > 0:
            return self.depth_scale
        # 16UC1 또는 uint16 깊이는 보통 mm 단위이므로 m로 바꿀 때 0.001을 곱합니다.
        if depth_image.dtype.name == 'uint16' or encoding == '16UC1':
            return 0.001
        # 32FC1처럼 이미 m 단위인 깊이는 그대로 사용합니다.
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
        # confidence 필터를 통과한 책 검출 결과를 저장할 목록입니다.
        detections = []
        # 입력 RGB 이미지 한 장을 YOLO 모델로 추론합니다.
        results = self.model(rgb_image, verbose=False)
        # YOLO가 반환한 결과 묶음을 순회합니다.
        for result in results:
            # segmentation mask가 있으면 mask 텐서를 가져옵니다.
            masks = result.masks.data if result.masks is not None else None
            # 결과 안의 모든 bounding box를 순회합니다.
            for index, box in enumerate(result.boxes):
                # 현재 상자의 confidence를 실수형으로 변환합니다.
                confidence = float(box.conf[0])
                # 설정된 최소 confidence보다 낮으면 버립니다.
                if confidence < self.confidence_threshold:
                    continue
                # 현재 상자의 class 번호를 읽습니다.
                class_id = int(box.cls[0])
                # class 번호를 모델의 class 이름으로 변환합니다.
                class_name = self.model.names[class_id]
                # 책이 아닌 객체는 후속 처리에서 제외합니다.
                if class_name != 'book':
                    continue
                # segmentation mask가 없을 때 사용할 기본값입니다.
                mask = None
                # 현재 상자에 대응하는 mask가 존재하는지 확인합니다.
                if masks is not None and index < len(masks):
                    # PyTorch mask를 NumPy boolean 배열로 변환합니다.
                    mask = masks[index].cpu().numpy() > 0.5
                    # mask 크기가 원본 영상과 다르면 영상 크기에 맞춥니다.
                    if mask.shape != rgb_image.shape[:2]:
                        # nearest 보간으로 mask의 0/1 값을 유지합니다.
                        mask = cv2.resize(
                            mask.astype(np.uint8),
                            (rgb_image.shape[1], rgb_image.shape[0]),
                            interpolation=cv2.INTER_NEAREST,
                        ).astype(bool)
                # BookDetector가 사용할 상자·mask·confidence를 저장합니다.
                detections.append({
                    # YOLO 상자 좌표를 정수 픽셀 tuple로 저장합니다.
                    'box': tuple(map(int, box.xyxy[0])),
                    # 책 영역 mask를 저장합니다.
                    'mask': mask,
                    # 검출 신뢰도를 저장합니다.
                    'confidence': confidence,
                })
        # 필터링된 책 검출 목록을 반환합니다.
        return detections



def main(args=None):
    # ROS 2 Python 통신을 초기화합니다.
    rclpy.init(args=args)

    # vision_manager 노드를 생성합니다.
    node = VisionManager()
    # action과 센서 callback을 병렬 처리할 executor를 생성합니다.
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    # executor에 노드를 등록합니다.
    executor.add_node(node)

    try:
        # 노드가 종료될 때까지 ROS callback을 계속 실행합니다.
        executor.spin()

    except KeyboardInterrupt:
        # Ctrl+C 종료를 로그로 남깁니다.
        node.get_logger().warn("강제 종료")

    finally:
        # OpenCV 창이 있다면 모두 닫습니다.
        cv2.destroyAllWindows()

        # executor를 종료하고 노드를 제거합니다.
        executor.shutdown()
        node.destroy_node()

        # ROS가 아직 실행 중이면 통신을 종료합니다.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    # 이 파일을 직접 실행했을 때 main 함수를 호출합니다.
    main()
