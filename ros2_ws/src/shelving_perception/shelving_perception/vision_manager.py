'''
[EN]
ROS 2 node that feeds Isaac Sim camera data to BookDetector.
ROS 2 node that feeds Isaac Sim camera data to TargetDetector.
[KR]
Isaac Sim 카메라 데이터를 BookDetector로 전달하는 ROS 2 노드.
Isaac Sim 카메라 데이터를 TargetDetector로 전달하는 ROS 2 노드.
'''

# ROS 2 Python 클라이언트 라이브러리입니다.
from pathlib import Path

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
from shelving_interfaces.action import DetectGraspPoint, DetectTargetSlot
# 향후 선반 슬롯 메시지에 사용할 타입입니다.
from shelving_interfaces.msg import GraspObservation
# ROS 2 노드의 기본 클래스입니다.
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
# RGB/Depth 영상과 카메라 내부 파라미터 메시지입니다.
from sensor_msgs.msg import Image, CameraInfo
# 책 위치와 책 pose를 발행할 메시지입니다.
from geometry_msgs.msg import PointStamped, PoseStamped
# **cv_bridge 를 쓰지 않습니다.** 이 파일 위쪽의 _imgmsg_to_* / _bgr8_to_imgmsg 로 대신합니다.
# cv_bridge 의 컴파일된 부분이 NumPy 1.x 로 빌드돼 있어 NumPy 2.x PC 에서 죽습니다.
# 카메라 좌표를 로봇 좌표로 변환하는 함수입니다.
from tf2_geometry_msgs import (
    do_transform_point,
    do_transform_pose_stamped,
)
# TF를 조회하고 TF 조회 실패를 처리하기 위한 클래스입니다.
from tf2_ros import Buffer, TransformException, TransformListener

# 깊이값을 이용해 책의 3차원 위치를 계산하는 보조 클래스입니다.
from .book_detector import BookDetector
# 깊이 영상에서 책장 빈 단을 찾는 보조 클래스입니다.
from .target_detector import TargetDetector


# sensor_msgs/Image 의 encoding → (numpy 자료형, 채널 수).
# cv_bridge 가 하던 일이지만, 여기 것은 순수 파이썬이라 OpenCV·NumPy 판과 무관합니다.
_ENCODINGS = {
    'rgb8': (np.uint8, 3), 'bgr8': (np.uint8, 3),
    'rgba8': (np.uint8, 4), 'bgra8': (np.uint8, 4),
    'mono8': (np.uint8, 1), '8UC1': (np.uint8, 1), '8UC3': (np.uint8, 3),
    'mono16': (np.uint16, 1), '16UC1': (np.uint16, 1),
    '32FC1': (np.float32, 1), '64FC1': (np.float64, 1),
}


def _imgmsg_to_array(msg):
    """sensor_msgs/Image → numpy 배열. **cv_bridge 를 거치지 않습니다**.

    `.1` 의 cv_bridge 는 컴파일된 `cvtColor2` 가 NumPy 1.x 로 빌드돼 있어,
    NumPy 2.x 환경에서 imgmsg_to_cv2 를 부르면 **세그폴트로 죽습니다**
    (2026-09-21 10.10.0.1, numpy 2.5.3). 바이트를 직접 해석하면 그 의존이 사라집니다.
    """
    entry = _ENCODINGS.get(msg.encoding)
    if entry is None:
        raise ValueError(f'모르는 encoding: {msg.encoding}')
    dtype, channels = entry
    dtype = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')
    array = np.frombuffer(bytearray(msg.data), dtype=dtype)
    # step 은 한 줄의 바이트 수입니다. 줄 끝 padding 이 있을 수 있으므로 step 으로 자릅니다
    array = array.reshape(msg.height, msg.step // dtype.itemsize)
    array = array[:, :msg.width * channels]
    return array.reshape(msg.height, msg.width) if channels == 1 \
        else array.reshape(msg.height, msg.width, channels)


def _imgmsg_to_bgr(msg):
    """sensor_msgs/Image → BGR 3채널 uint8. 색 변환은 cv2 로 합니다."""
    array = _imgmsg_to_array(msg)
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    if msg.encoding == 'rgba8':
        return cv2.cvtColor(array, cv2.COLOR_RGBA2BGR)
    if msg.encoding == 'bgra8':
        return cv2.cvtColor(array, cv2.COLOR_BGRA2BGR)
    if msg.encoding in ('mono8', '8UC1'):
        return cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)
    if msg.encoding in ('bgr8', '8UC3'):
        return array
    raise ValueError(f'BGR 로 바꿀 수 없는 encoding: {msg.encoding}')


def _bgr8_to_imgmsg(image):
    """BGR numpy 배열 → sensor_msgs/Image. **cv_bridge 를 거치지 않습니다.**

    cv_bridge.cv2_to_imgmsg 는 OpenCV 5 에서 KeyError 로 죽습니다. OpenCV 5 가
    CV_CN_SHIFT 를 3 에서 5 로 바꿔, cv_bridge 가 만든 타입 표(CV_8UC3 = 64)와
    자체 계산값(16)이 어긋나기 때문입니다. bgr8 은 메모리 배치가 그대로이므로
    직접 채우면 OpenCV 판 번호와 무관하게 동작합니다.
    """
    image = np.ascontiguousarray(image, dtype=np.uint8)
    height, width = image.shape[:2]
    msg = Image()
    msg.height = int(height)
    msg.width = int(width)
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = int(width * 3)
    msg.data = image.tobytes()
    return msg


class VisionManager(Node):
    """카메라 영상을 받아 책을 검출하고 ROS 결과로 변환하는 노드입니다."""

    def __init__(self):
        # 노드 이름을 vision_manager로 등록합니다.
        super().__init__('vision_manager')

        # RGB 영상이 들어오는 토픽 이름을 파라미터에서 읽습니다.
        self.rgb_topic = self.declare_parameter(
            'rgb_topic', '/rgb').value
        # 깊이 영상이 들어오는 토픽 이름을 파라미터에서 읽습니다.
        self.depth_topic = self.declare_parameter(
            'depth_topic', '/depth').value
        # 카메라 내부 파라미터가 들어오는 토픽 이름을 읽습니다.
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/camera_info').value
        # Manipulation이 요청할 책 검출 및 빈 슬롯 검출 action 이름을 읽습니다.
        self.grasp_action = self.declare_parameter("grasp_action", "/detect_grasp_point").value
        self.slot_action = self.declare_parameter("slot_action", "/detect_target_slot").value
        self.action_timeout_s = float(self.declare_parameter("action_timeout_s", 15.0).value)
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
        # 검출 상자와 confidence를 그린 디버그 영상을 발행할 토픽입니다.
        self.debug_image_topic = self.declare_parameter(
            'debug_image_topic', '/perception/debug_image').value
        # 책장 빈 단의 3D 점을 발행할 토픽입니다.
        self.empty_slot_topic = self.declare_parameter(
            'empty_slot_topic', '/perception/empty_shelf_position').value
        # 패키지 resource 폴더에 포함된 YOLO weight의 기본 경로입니다.
        try:
            resource_dir = (
                Path(get_package_share_directory('shelving_perception'))
                / 'resource'
            )
        except PackageNotFoundError:
            resource_dir = Path(__file__).resolve().parent.parent / 'resource'
        default_model_path = str(resource_dir / 'book_tray_best.pt')
        default_shelf_model_path = str(resource_dir / 'best.pt')
        # 책 검출 모델의 경로입니다.
        self.model_path = self.declare_parameter(
            'model_path', default_model_path).value
        # 책장 segmentation 모델의 경로입니다.
        self.shelf_model_path = self.declare_parameter(
            'shelf_model_path', default_shelf_model_path).value
        # 책장 검출을 인정할 최소 confidence입니다.
        self.shelf_confidence_threshold = float(self.declare_parameter(
            'shelf_confidence_threshold', 0.50).value)
        # 책 ROI를 카메라에 투영할 때 사용하는 로봇 기준 frame입니다.
        self.slot_position_frame = self.declare_parameter(
            'slot_position_frame', 'arm_base_link').value
        # 책장 안의 연속된 빈 영역을 찾을 때 사용하는 깊이 샘플 설정입니다.
        self.target_sample_radius_px = int(self.declare_parameter(
            'target_sample_radius_px', 5).value)
        self.target_side_offset_px = int(self.declare_parameter(
            'target_side_offset_px', 25).value)
        self.target_min_depth_difference = float(self.declare_parameter(
            'target_min_depth_difference', 0.05).value)
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
        # 검출된 선반 앞면에서 책 중심이 들어갈 만큼 안쪽으로 보정합니다.
        self.slot_center_inset = float(self.declare_parameter(
            'slot_center_inset', 0.024).value)
        # 서가 바닥에서 책 AABB 중심을 이만큼 띄운다.
        # book_scene.plan_job의 grip_z 계약과 같은 4 mm이다.
        self.slot_floor_clearance = float(self.declare_parameter(
            'slot_floor_clearance', 0.004).value)
        # 삽입 전 대기 위치의 오프셋입니다.
        self.pre_insert_offset = float(self.declare_parameter(
            'pre_insert_offset', 0.05).value)

        # 현재 시뮬레이션에서 사용하는 표준 책의 실제 치수입니다.
        # 책 종류가 다양해지면 검출 결과 또는 별도 프로파일로 교체합니다.
        self.default_book_thickness = float(self.declare_parameter(
            'default_book_thickness', 0.0353).value)
        self.default_book_height = float(self.declare_parameter(
            'default_book_height', 0.2374).value)
        self.default_book_width = float(self.declare_parameter(
            'default_book_width', 0.1631).value)

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
        # 접근 가능한 ROI 안에 고신뢰도 후보가 없을 때만 사용하는
        # 보조 confidence입니다. 양 끝 책은 이후 ROI 필터에서 계속 제외됩니다.
        self.fallback_confidence_threshold = float(self.declare_parameter(
            'fallback_confidence_threshold', 0.20).value)
        if (
            self.fallback_confidence_threshold <= 0.0
            or self.fallback_confidence_threshold > self.confidence_threshold
        ):
            raise ValueError(
                'fallback_confidence_threshold must be greater than 0 '
                'and no greater than confidence_threshold.'
            )
        # 신뢰도 1등 한 권만 발행·표시할지. false 면 검출된 전부 (디버그용)
        self.publish_best_only = bool(self.declare_parameter(
            'publish_best_only', True).value)
        # **책 검출 ROI — 트레이 영역만 본다.** 팔 기준(arm_base_link) 3D 상자다.
        # 픽셀 ROI 가 아니라 3D 로 두는 이유: 카메라가 움직여도 같은 영역을 가리킨다.
        # **책장 빈 공간 경로에는 적용하지 않는다** (거기는 ROI 가 있으면 안 된다).
        # **가로(x)는 트레이에 딱 맞춘다.** 넓게 두면 로봇팔 받침대나 바닥이
        # 책으로 잡히는 일이 있다 (2026-09-21 현장 관찰).
        #   칸 중심   -0.6623 ~ -0.2873   (6칸, 간격 0.075)
        #   책 바깥면 -0.6800 ~ -0.2697   (두께 0.0353 의 절반을 더함)
        #   여유 10mm -0.690  ~ -0.260    ← 이것을 쓴다
        # y·z 는 깊이·높이라 여유를 둔다 (책 윗면 z ~0.1935).
        self.book_roi_enabled = bool(self.declare_parameter(
            'book_roi_enabled', True).value)
        self.book_roi_min = [float(v) for v in self.declare_parameter(
            'book_roi_min', [-0.69, -0.02, 0.00]).value]
        self.book_roi_max = [float(v) for v in self.declare_parameter(
            'book_roi_max', [-0.26, 0.18, 0.32]).value]

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
        if self.slot_center_inset < 0.0:
            raise ValueError('slot_center_inset must be non-negative')
        if self.slot_floor_clearance < 0.0:
            raise ValueError('slot_floor_clearance must be non-negative')

        # 학습된 YOLO 모델을 메모리에 로드합니다.
        self.model = YOLO(self.model_path)
        # 책장 segmentation YOLO 모델을 별도로 로드합니다.
        self.shelf_model = YOLO(self.shelf_model_path)
        # 검출 상자와 깊이로 3차원 좌표를 계산하는 객체를 생성합니다.
        self.book_detector = BookDetector()
        # 책장 각 단의 빈 공간을 깊이로 판단하는 객체를 생성합니다.
        self.target_detector = TargetDetector(
            sample_radius_px=self.target_sample_radius_px,
            side_offset_px=self.target_side_offset_px,
            min_side_depth_difference=self.target_min_depth_difference,
        )
        # TF 변환을 조회할 버퍼와 listener를 생성합니다.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # action callback이 센서 callback과 동시에 실행될 수 있도록 그룹을 만듭니다.

        self._action_group = ReentrantCallbackGroup()
        self._action_lock = threading.Lock()
        self._action_event = threading.Event()

        self._active_goal = None
        self._active_kind = None
        self._action_result = None
        self._goal_reserved = False
        self.pending_detection = False

        self._grasp_action_server = ActionServer(
            self,
            DetectGraspPoint,
            self.grasp_action,
            execute_callback=self._execute_grasp_detection,
            goal_callback=self._accept_detection_goal,
            cancel_callback=self._cancel_detection_goal,
            callback_group=self._action_group,
        )

        self._slot_action_server = ActionServer(
            self,
            DetectTargetSlot,
            self.slot_action,
            execute_callback=self._execute_slot_detection,
            goal_callback=self._accept_detection_goal,
            cancel_callback=self._cancel_detection_goal,
            callback_group=self._action_group,
        )

        self.book_pub = self.create_publisher(
            PointStamped,
            self.book_topic,
            10,
        )

        self.book_pose_pub = self.create_publisher(
            PoseStamped,
            self.book_pose_topic,
            10,
        )

        self.empty_slot_pub = self.create_publisher(
            PointStamped,
            self.empty_slot_topic,
            10,
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            self.debug_image_topic,
            10,
        )

        # Isaac 카메라는 렌더 프레임마다 대용량 메시지를 발행합니다. 기본
        # reliable 큐를 사용하면 Python 검출기가 처리하는 동안 과거 영상이
        # 누적되어, action의 not_before 시각보다 오래된 프레임만 계속 받게
        # 됩니다. 검출에는 기록 보존보다 최신 프레임이 중요하므로 센서 QoS로
        # 한 장만 유지합니다.
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.rgb_sub = message_filters.Subscriber(
            self,
            Image,
            self.rgb_topic,
            qos_profile=sensor_qos,
        )

        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=sensor_qos,
        )

        self.camera_info_sub = message_filters.Subscriber(
            self,
            CameraInfo,
            self.camera_info_topic,
            qos_profile=sensor_qos,
        )

        self.image_sync = message_filters.ApproximateTimeSynchronizer(
            [
                self.rgb_sub,
                self.depth_sub,
                self.camera_info_sub,
            ],
            queue_size=3,
            slop=self.sync_slop,
        )

        self.image_sync.registerCallback(self.rgb_callback)

        self.get_logger().info(
            "Perception action servers ready: "
            f"grasp={self.grasp_action}, "
            f"slot={self.slot_action}"
        )

    def _accept_detection_goal(self, goal_request):
        """두 perception action 중 하나만 실행되도록 제한합니다."""
        with self._action_lock:
            if self._goal_reserved or self._active_goal is not None:
                self.get_logger().warning(
                    "Another perception request is already running."
                )
                return GoalResponse.REJECT

            self._goal_reserved = True

        return GoalResponse.ACCEPT

    def _cancel_detection_goal(self, goal_handle):
        """진행 중인 perception action의 취소를 허용합니다."""
        return CancelResponse.ACCEPT

    def _execute_grasp_detection(self, goal_handle):
        """트레이 위 책 검출 action을 실행합니다."""
        return self._execute_detection(goal_handle, "grasp")

    def _execute_slot_detection(self, goal_handle):
        """책장의 빈 슬롯 검출 action을 실행합니다."""
        return self._execute_detection(goal_handle, "slot")

    def _new_action_result(self, action_kind):
        """현재 action 종류에 맞는 Result 메시지를 생성합니다."""
        if action_kind == "grasp":
            return DetectGraspPoint.Result()

        return DetectTargetSlot.Result()

    def _clear_action_state(self):
        """현재 perception action의 공유 상태를 초기화합니다."""
        with self._action_lock:
            self._active_goal = None
            self._active_kind = None
            self._action_result = None
            self._goal_reserved = False
            self.pending_detection = False
            self._action_event.clear()

    def _execute_detection(self, goal_handle, action_kind):
        """다음 유효한 카메라 프레임의 검출 결과를 기다립니다."""
        with self._action_lock:
            self._active_goal = goal_handle
            self._active_kind = action_kind
            self._action_result = None
            self._action_event.clear()
            self.pending_detection = True

        self._publish_action_feedback(
            goal_handle,
            "CAPTURING",
            0,
            0.0,
        )

        started = time.monotonic()

        while not self._action_event.wait(0.1):
            if goal_handle.is_cancel_requested:
                result = self._new_action_result(action_kind)
                result.success = False
                result.error_code = 3002
                result.message = "Perception request was canceled."

                self._clear_action_state()
                goal_handle.canceled()
                return result

            if time.monotonic() - started > self.action_timeout_s:
                result = self._new_action_result(action_kind)
                result.success = False
                result.error_code = 3003
                result.message = (
                    "Timed out waiting for a valid camera frame."
                )

                self._clear_action_state()
                goal_handle.abort()
                return result

        with self._action_lock:
            result = self._action_result

        if result is None:
            result = self._new_action_result(action_kind)
            result.success = False
            result.error_code = 3004
            result.message = "Perception returned no result."

            self._clear_action_state()
            goal_handle.abort()
            return result

        self._clear_action_state()

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def _publish_action_feedback(
        self,
        goal_handle,
        phase,
        count,
        confidence,
    ):
        """현재 실행 중인 action 형식으로 feedback을 발행합니다."""
        with self._action_lock:
            action_kind = self._active_kind

        if action_kind == "grasp":
            feedback = DetectGraspPoint.Feedback()
        elif action_kind == "slot":
            feedback = DetectTargetSlot.Feedback()
        else:
            return

        feedback.phase = phase
        feedback.candidate_count = count
        feedback.best_confidence = float(confidence)

        goal_handle.publish_feedback(feedback)

    def rgb_callback(self, rgb_msg, depth_msg, camera_info_msg):
        """현재 실행 중인 perception action에 필요한 검출만 수행합니다."""
        with self._action_lock:
            if not self.pending_detection:
                return

            active_goal = self._active_goal
            action_kind = self._active_kind

        if active_goal is None or action_kind is None:
            return

        if not rgb_msg.header.frame_id or not depth_msg.header.frame_id:
            self.get_logger().warning(
                "RGB or depth frame_id is empty."
            )
            return

        if (
            rgb_msg.header.frame_id != self.camera_frame
            or depth_msg.header.frame_id != self.camera_frame
            or camera_info_msg.header.frame_id != self.camera_frame
        ):
            self.get_logger().warning(
                f"Expected camera frame {self.camera_frame}, got "
                f"rgb={rgb_msg.header.frame_id}, "
                f"depth={depth_msg.header.frame_id}, "
                f"camera_info={camera_info_msg.header.frame_id}"
            )
            return

        if (
            rgb_msg.header.frame_id != depth_msg.header.frame_id
            and not self.depth_registered
        ):
            self.get_logger().warning(
                "RGB and depth frames differ, but depth_registered is false."
            )
            return

        # 로봇팔이 관측 위치에 도달하기 전에 촬영된 프레임은 사용하지 않습니다.
        frame_stamp = rgb_msg.header.stamp
        frame_stamp_ns = (
            int(frame_stamp.sec) * 1_000_000_000
            + int(frame_stamp.nanosec)
        )

        not_before = active_goal.request.not_before
        not_before_ns = (
            int(not_before.sec) * 1_000_000_000
            + int(not_before.nanosec)
        )

        if (
            not_before_ns > 0
            and frame_stamp_ns < not_before_ns
        ):
            return

        try:
            rgb_image = _imgmsg_to_bgr(rgb_msg)
            depth_image = _imgmsg_to_array(depth_msg)
        except ValueError as error:
            self.get_logger().warning(
                f"Image conversion failed: {error}"
            )
            return

        if rgb_image.shape[:2] != depth_image.shape[:2]:
            self.get_logger().warning(
                "RGB and depth image sizes differ."
            )
            return

        info = camera_info_msg
        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])

        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().warning(
                "Invalid camera intrinsics in CameraInfo."
            )
            return

        depth_scale = self._get_depth_scale(
            depth_msg.encoding,
            depth_image,
        )

        # 유효한 프레임 하나를 확보했으므로 중복 처리를 막습니다.
        with self._action_lock:
            if self._active_goal is not active_goal:
                return

            self.pending_detection = False

        try:
            if action_kind == "grasp":
                self._process_grasp_frame(
                    active_goal,
                    rgb_image,
                    depth_image,
                    rgb_msg.header,
                    fx,
                    fy,
                    cx,
                    cy,
                    depth_scale,
                )
                return

            if action_kind == "slot":
                self._process_slot_frame(
                    active_goal,
                    rgb_image,
                    depth_image,
                    rgb_msg.header,
                    fx,
                    fy,
                    cx,
                    cy,
                    depth_scale,
                )
                return

            self._finish_action_with_failure(
                3004,
                f"Unknown perception action kind: {action_kind}",
            )

        except Exception as error:
            self.get_logger().error(
                f"{action_kind} detection failed: "
                f"{type(error).__name__}: {error}"
            )
            self._finish_action_with_failure(
                3003,
                f"{action_kind} detection failed: {error}",
            )

    def _process_grasp_frame(
        self,
        goal_handle,
        rgb_image,
        depth_image,
        header,
        fx,
        fy,
        cx,
        cy,
        depth_scale,
    ):
        """트레이 영상에서 책 후보만 검출합니다."""
        detections = self._detect_books(rgb_image)

        projected_targets = self.book_detector.process(
            detections,
            depth_image,
            fx,
            fy,
            cx,
            cy,
            depth_scale,
        )

        reachable_targets, roi_dropped = self._filter_book_roi(
            projected_targets,
            header.frame_id,
            header.stamp,
        )

        strong_targets = [
            target for target in reachable_targets
            if float(target['confidence']) >= self.confidence_threshold
        ]
        if strong_targets:
            detected_targets = strong_targets
            selection_mode = 'primary'
        else:
            detected_targets = [
                target for target in reachable_targets
                if float(target['confidence'])
                >= self.fallback_confidence_threshold
            ]
            selection_mode = 'fallback' if detected_targets else 'none'

        if selection_mode == 'fallback':
            self.get_logger().warning(
                'No reachable book met primary confidence '
                f'{self.confidence_threshold:.2f}; using ROI-constrained '
                'fallback candidates at or above '
                f'{self.fallback_confidence_threshold:.2f}.'
            )

        best_confidence = max(
            (
                float(target["confidence"])
                for target in detected_targets
            ),
            default=0.0,
        )

        self._publish_action_feedback(
            goal_handle,
            "DETECTING_BOOK",
            len(detected_targets),
            best_confidence,
        )

        self._publish_debug_image(
            rgb_image,
            header,
            detections,
            roi_px=self._book_roi_pixels(
                header.frame_id,
                header.stamp,
                fx,
                fy,
                cx,
                cy,
                rgb_image.shape,
            ),
            keep_boxes=[
                target["box"]
                for target in detected_targets
            ],
            dropped=roi_dropped,
        )

        self.get_logger().info(
            'Book candidates: '
            f'detections={len(detections)}, '
            f'projected={len(projected_targets)}, '
            f'reachable={len(reachable_targets)}, '
            f'valid={len(detected_targets)}, '
            f'selection_mode={selection_mode}, '
            f'roi_dropped={roi_dropped}, '
            f'best_confidence={best_confidence:.3f}'
        )

        self._complete_action_from_books(
            goal_handle,
            detected_targets,
            header.frame_id,
            header.stamp,
        )

    def _process_slot_frame(
        self,
        goal_handle,
        rgb_image,
        depth_image,
        header,
        fx,
        fy,
        cx,
        cy,
        depth_scale,
    ):
        """책장 영상에서 고정 후보 없이 연속된 빈 영역을 검출합니다."""
        shelf_detection = self._detect_shelf(rgb_image)
        inspection = self.target_detector.find_empty_position(
            depth_image,
            shelf_detection['box'] if shelf_detection is not None else None,
            fx,
            fy,
            cx,
            cy,
            depth_scale,
        )
        is_empty = bool(inspection and inspection.get('is_empty'))

        self._publish_action_feedback(
            goal_handle,
            "DETECTING_SLOT",
            int(is_empty),
            1.0 if is_empty else 0.0,
        )

        self._publish_debug_image(
            rgb_image,
            header,
            [],
            shelf_detection,
            inspection,
        )
        empty_point = None
        if is_empty:
            empty_point = self._project_slot_to_shelf_plane(
                inspection,
                header.frame_id,
                header.stamp,
                goal_handle.request,
            )
            if empty_point is None:
                # shelf_plane_y/shelf_floor_z가 없는 독립 비전 테스트만
                # 기존 depth 기반 경로로 돌아간다. 운영 통합에서는
                # manipulation executor가 두 값을 반드시 보낸다.
                empty_point = self._transform_xyz(
                    inspection['target_xyz'],
                    header.frame_id,
                    header.stamp,
                )
            if empty_point is not None:
                self.empty_slot_pub.publish(empty_point)
                self.get_logger().info(
                    "Empty shelf target: "
                    f"xyz=({empty_point.point.x:.3f}, "
                    f"{empty_point.point.y:.3f}, "
                    f"{empty_point.point.z:.3f})"
                )

        self._complete_action_from_empty_position(goal_handle, empty_point)

    def _detect_shelf(self, rgb_image):
        """책장 YOLO 모델에서 가장 신뢰도 높은 책장 검출을 반환합니다."""
        # 책장 모델의 결과를 저장할 후보 목록입니다.
        candidates = []
        # RGB 이미지에서 책장 segmentation을 수행합니다.
        results = self.shelf_model(rgb_image, verbose=False)
        # 모델 결과 묶음을 순회합니다.
        for result in results:
            # 현재 결과의 모든 bounding box를 순회합니다.
            for box in result.boxes:
                # 책장 검출 confidence를 실수로 변환합니다.
                confidence = float(box.conf[0])
                # 낮은 confidence 결과는 제거합니다.
                if confidence < self.shelf_confidence_threshold:
                    continue
                # 모델 class 번호를 읽습니다.
                class_id = int(box.cls[0])
                # class 번호를 shelf_open 등의 이름으로 변환합니다.
                class_name = self.shelf_model.names[class_id]
                # 책장 클래스가 아니면 무시합니다.
                if not class_name.startswith('shelf_'):
                    continue
                # 책장 bbox를 정수 픽셀 좌표로 저장합니다.
                shelf_box = tuple(map(int, box.xyxy[0]))
                # 후보를 confidence 순서로 선택할 수 있게 저장합니다.
                candidates.append({
                    'box': shelf_box,
                    'class_name': class_name,
                    'confidence': confidence,
                })

        # 검출된 책장이 없으면 None을 반환합니다.
        if not candidates:
            return None
        # 가장 신뢰도가 높은 책장 하나를 선택합니다.
        return max(candidates, key=lambda candidate: candidate['confidence'])

    def _arm_to_camera_tf(self, camera_frame, stamp):
        """팔 기준 → 카메라 기준 TF. 못 구하면 None."""
        try:
            return self.tf_buffer.lookup_transform(
                camera_frame,
                self.slot_position_frame,
                rclpy.time.Time.from_msg(stamp),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except TransformException:
            return None

    def _filter_book_roi(self, targets, camera_frame, stamp):
        """**트레이 ROI 안의 책만** 남깁니다. (남은 것, 버린 수) 를 돌려줍니다.

        왜 3D 인가: 픽셀 상자로 자르면 카메라가 조금만 움직여도 엉뚱한 영역을 가립니다.
        팔 기준 상자는 카메라가 어디서 보든 **같은 실제 공간**을 가리킵니다.

        **책장 빈 공간 판정에는 쓰지 않습니다.** 거기는 ROI 가 있으면 안 됩니다 —
        서가 어디든 빈 칸이 생길 수 있기 때문입니다.
        """
        if not self.book_roi_enabled or not targets:
            return targets, 0
        kept, dropped = [], 0
        for target in targets:
            point = self._transform_xyz(target['xyz'], camera_frame, stamp)
            if point is None:
                # 변환 실패는 **버리지 않습니다** — ROI 판정을 못 한 것이지
                # 밖에 있다고 밝혀진 것이 아닙니다
                kept.append(target)
                continue
            p = point.point
            lo, hi = self.book_roi_min, self.book_roi_max
            if (lo[0] <= p.x <= hi[0] and lo[1] <= p.y <= hi[1]
                    and lo[2] <= p.z <= hi[2]):
                kept.append(target)
            else:
                dropped += 1
        if dropped:
            self.get_logger().info(
                f'ROI 밖 책 {dropped}권 제외 (남은 {len(kept)}권). '
                f'ROI {self.book_roi_min} ~ {self.book_roi_max} @'
                f'{self.slot_position_frame}')
        return kept, dropped

    def _book_roi_pixels(self, camera_frame, stamp, fx, fy, cx, cy, shape):
        """ROI 3D 상자를 화면 사각형으로 투영합니다. 못 구하면 None."""
        if not self.book_roi_enabled:
            return None
        transform = self._arm_to_camera_tf(camera_frame, stamp)
        if transform is None:
            return None
        lo, hi = self.book_roi_min, self.book_roi_max
        us, vs = [], []
        for x in (lo[0], hi[0]):
            for y in (lo[1], hi[1]):
                for z in (lo[2], hi[2]):
                    pt = PointStamped()
                    pt.header.frame_id = self.slot_position_frame
                    pt.point.x, pt.point.y, pt.point.z = x, y, z
                    c = do_transform_point(pt, transform).point
                    if c.z <= 1e-6:          # 카메라 뒤쪽은 투영할 수 없다
                        continue
                    us.append(c.x * fx / c.z + cx)
                    vs.append(c.y * fy / c.z + cy)
        if len(us) < 2:
            return None
        h, w = shape[:2]
        return (max(0, int(min(us))), max(0, int(min(vs))),
                min(w - 1, int(max(us))), min(h - 1, int(max(vs))))

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

    def _project_slot_to_shelf_plane(
        self,
        inspection,
        source_frame,
        stamp,
        request,
    ):
        """빈칸 픽셀의 카메라 광선을 실제 서가 앞면과 교차시킵니다.

        빈칸 중앙의 depth는 서가 뒤쪽 벽입니다. 그 값을 삽입
        좌표로 쓰면 2~3 m 뒤의 서가가 목표가 됩니다. depth는
        광선 방향을 정하는 데만 쓰고, 삽입 깊이와 단 높이는
        Isaac이 USD에서 계산해 action goal로 보낸 값을 사용합니다.
        """
        plane_y = float(request.shelf_plane_y)
        floor_z = float(request.shelf_floor_z)

        if (
            not math.isfinite(plane_y)
            or not math.isfinite(floor_z)
            or plane_y <= 0.0
        ):
            return None

        ray_camera = inspection.get('camera_xyz')
        if ray_camera is None:
            return None

        origin = self._transform_xyz(
            (0.0, 0.0, 0.0),
            source_frame,
            stamp,
        )
        ray_point = self._transform_xyz(
            ray_camera,
            source_frame,
            stamp,
        )
        if origin is None or ray_point is None:
            return None

        direction = np.array([
            ray_point.point.x - origin.point.x,
            ray_point.point.y - origin.point.y,
            ray_point.point.z - origin.point.z,
        ], dtype=float)

        if abs(float(direction[1])) < 1e-6:
            self.get_logger().warning(
                'Empty-slot camera ray is parallel to the shelf plane.'
            )
            return None

        distance = (
            plane_y - float(origin.point.y)
        ) / float(direction[1])
        if not math.isfinite(distance) or distance <= 0.0:
            self.get_logger().warning(
                'Empty-slot camera ray does not point toward the shelf plane.'
            )
            return None

        point = PointStamped()
        point.header.frame_id = self.target_frame
        point.header.stamp = stamp
        point.point.x = float(origin.point.x + distance * direction[0])
        point.point.y = plane_y
        point.point.z = (
            floor_z
            + max(0.0, float(request.book_height)) * 0.5
            + self.slot_floor_clearance
        )

        self.get_logger().info(
            'Empty-slot plane projection: '
            f'raw=({ray_point.point.x:.3f}, '
            f'{ray_point.point.y:.3f}, '
            f'{ray_point.point.z:.3f}), '
            f'plane_y={plane_y:.3f}, floor_z={floor_z:.3f}, '
            f'projected=({point.point.x:.3f}, '
            f'{point.point.y:.3f}, '
            f'{point.point.z:.3f})'
        )
        return point

    def _publish_debug_image(self, *args, **kwargs):
        """디버그 영상을 발행합니다. **그리다 실패해도 검출을 멈추지 않습니다**.

        화면 표시는 곁가지인데, 여기서 난 예외가 노드를 통째로 내려버린 적이 있습니다
        (2026-09-21 GPU PC: KeyError 'row_index' → vision_manager 종료 → 파지 중단).
        검출·좌표 발행은 화면과 무관하게 계속되어야 합니다.
        """
        try:
            self._draw_and_publish_debug_image(*args, **kwargs)
        except Exception as error:       # noqa: BLE001 - 화면 때문에 노드가 죽으면 안 됩니다
            self.get_logger().warning(
                f'디버그 영상 표시 실패(검출은 계속합니다): {type(error).__name__}: {error}',
                throttle_duration_sec=10.0)

    def _draw_and_publish_debug_image(
        self,
        rgb_image,
        header,
        detections,
        shelf_detection=None,
        empty_slots=None,
        roi_px=None,
        keep_boxes=None,
        dropped=0,
    ):
        """책장·책·빈 단 표시를 그린 영상을 ROS Image로 발행합니다."""
        # 원본 영상을 복사해 디버그 표시가 원본 데이터에 영향을 주지 않게 합니다.
        debug_image = rgb_image.copy()
        # **트레이 ROI** 를 노란 상자로 그립니다 (팔 기준 3D 상자를 화면에 투영한 것).
        if roi_px is not None:
            rx1, ry1, rx2, ry2 = roi_px
            cv2.rectangle(debug_image, (rx1, ry1), (rx2, ry2), (0, 255, 255), 2)
            cv2.putText(
                debug_image,
                f"tray ROI (dropped {dropped})",
                (rx1 + 4, max(18, ry1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        # ROI 밖이라 버린 책은 **회색 얇은 상자**로 남깁니다 — 왜 안 골랐는지 보이게
        if keep_boxes is not None:
            kept = {tuple(b) for b in keep_boxes}
            for det in detections:
                if tuple(det['box']) not in kept:
                    bx1, by1, bx2, by2 = det['box']
                    cv2.rectangle(
                        debug_image, (bx1, by1), (bx2, by2), (150, 150, 150), 1)
            detections = [d for d in detections if tuple(d['box']) in kept]
        # **신뢰도 1등 한 권만** 그립니다 (로봇팔에 보내는 그 한 권).
        draw_list = detections
        if detections and getattr(self, 'publish_best_only', True):
            draw_list = [max(detections, key=lambda d: d['confidence'])]
        for detection in draw_list:
            # 검출 상자의 픽셀 좌표를 읽습니다.
            x1, y1, x2, y2 = detection['box']
            # 책 상자를 초록색 사각형으로 표시합니다.
            cv2.rectangle(
                debug_image,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2,
            )
            # **로봇팔에 보내는 좌표가 바로 이 점**이다 — 눈으로 확인할 수 있게 찍는다
            cu, cv_ = detection.get('center', ((x1 + x2) // 2, (y1 + y2) // 2))
            cv2.circle(debug_image, (int(cu), int(cv_)), 5, (0, 0, 255), -1)
            # 화면에 표시할 confidence 문자열을 만듭니다.
            label = f"book {detection['confidence']:.2f}"
            # 상자 위쪽에 검출 class와 confidence를 표시합니다.
            cv2.putText(
                debug_image,
                label,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        # 책장 검출 결과가 있으면 파란색 bbox로 표시합니다.
        if shelf_detection is not None:
            x1, y1, x2, y2 = shelf_detection['box']
            cv2.rectangle(
                debug_image,
                (x1, y1),
                (x2, y2),
                (255, 0, 0),
                2,
            )
            cv2.putText(
                debug_image,
                f"{shelf_detection['class_name']} "
                f"{shelf_detection['confidence']:.2f}",
                (x1, min(debug_image.shape[0] - 10, y2 + 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 0, 0),
                2,
                cv2.LINE_AA,
            )
        # 빈 단의 중심 픽셀을 빨간색 원으로 표시합니다.
        # **없는 열쇠에 죽지 않습니다.** 빈 단 정보의 구성은 경로마다 다른데,
        # 그림을 그리다 KeyError 로 노드 전체가 내려간 적이 있습니다
        # (2026-09-21 GPU PC: KeyError 'row_index' 로 vision_manager 종료 → 파지 중단).
        slot_items = (
            [empty_slots] if isinstance(empty_slots, dict)
            else (empty_slots or [])
        )
        for empty_slot in slot_items:
            pixel = empty_slot.get('pixel')
            if pixel is None:
                continue
            u, v = pixel
            cv2.circle(debug_image, (u, v), 8, (0, 0, 255), -1)
            cv2.putText(
                debug_image,
                "empty target",
                (u + 10, v),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
        # OpenCV BGR 영상을 ROS Image 메시지로 변환합니다.
        # **cv_bridge 를 쓰지 않습니다.** OpenCV 5 에서 CV_CN_SHIFT 가 3→5 로 바뀌어,
        # cv_bridge 의 타입 표(키 32..36)와 자체 계산값(16)이 어긋나 cv2_to_imgmsg 가
        # KeyError 로 죽습니다 (2026-09-21 GPU PC 10.10.0.1, cv2 5.0.0 에서 확인).
        # bgr8 은 바이트를 그대로 옮기면 되므로 직접 만듭니다 — OpenCV 판 번호와 무관합니다.
        debug_msg = _bgr8_to_imgmsg(debug_image)
        # 원본 RGB와 같은 timestamp/frame을 유지합니다.
        debug_msg.header = header
        # 다른 PC의 rqt_image_view가 구독할 수 있도록 발행합니다.
        self.debug_image_pub.publish(debug_msg)
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

    def _complete_action_from_empty_position(
        self,
        goal_handle,
        point,
    ):
        """연속 빈 영역에서 계산한 삽입 중심 하나를 반환합니다."""
        result = DetectTargetSlot.Result()
        result.success = False
        result.candidate_count = int(point is not None)

        if point is None:
            result.error_code = 3003
            result.message = "No empty shelf position was detected."
            self._set_action_result(result)
            return

        request = goal_handle.request
        book_half_depth = max(0.0, float(request.book_width)) * 0.5

        self._publish_action_feedback(
            goal_handle,
            "CALCULATING_POSE",
            1,
            1.0,
        )

        slot = result.target_slot
        slot.header = point.header

        slot.pose.position.x = point.point.x
        slot.pose.position.y = (
            point.point.y + book_half_depth + self.slot_center_inset
        )
        slot.pose.position.z = point.point.z

        # 책장 삽입 방향은 현재 검증된 +90도 yaw를 유지합니다.
        slot.pose.orientation.x = 0.0
        slot.pose.orientation.y = 0.0
        slot.pose.orientation.z = math.sqrt(0.5)
        slot.pose.orientation.w = math.sqrt(0.5)

        # 연속 깊이 기반 검출은 고정 슬롯의 폭·높이를 가정하지 않습니다.
        slot.available_width = 0.0
        slot.available_height = 0.0
        slot.insertion_depth = self.insertion_depth
        slot.pre_insert_offset = self.pre_insert_offset
        slot.confidence = 1.0

        result.success = True
        result.error_code = 0
        result.message = "Empty shelf position detected."

        self._publish_action_feedback(
            goal_handle,
            "TRANSFORMING_FRAME",
            1,
            slot.confidence,
        )

        self._set_action_result(result)

    def _complete_action_from_books(
        self,
        goal_handle,
        detected_targets,
        source_frame,
        stamp,
    ):
        """가장 신뢰도가 높은 책을 GraspObservation으로 반환합니다."""
        result = DetectGraspPoint.Result()
        result.success = False
        result.candidate_count = len(detected_targets)

        if not detected_targets:
            result.error_code = 3003
            result.message = "No book candidate was detected."
            self._set_action_result(result)
            return

        target = max(
            detected_targets,
            key=lambda candidate: candidate["confidence"],
        )

        point = self._transform_xyz(
            target["xyz"],
            source_frame,
            stamp,
        )

        if point is None:
            result.error_code = 3004
            result.message = (
                "Could not transform detected book coordinates."
            )
            self._set_action_result(result)
            return

        book_pose = self._make_book_pose(
            target["xyz"],
            source_frame,
            stamp,
            target["image_angle"],
        )

        if book_pose is None:
            result.error_code = 3004
            result.message = (
                "Could not transform detected book orientation."
            )
            self._set_action_result(result)
            return

        self._publish_action_feedback(
            goal_handle,
            "CALCULATING_POSE",
            len(detected_targets),
            target["confidence"],
        )

        q = book_pose.pose.orientation

        sin_yaw = 2.0 * (
            q.w * q.z
            + q.x * q.y
        )
        cos_yaw = 1.0 - 2.0 * (
            q.y * q.y
            + q.z * q.z
        )
        spine_yaw = math.atan2(sin_yaw, cos_yaw)

        grasp = result.grasp
        grasp.header = point.header
        grasp.top_center = point.point
        grasp.spine_yaw = float(spine_yaw)

        # 0이면 책 치수를 확정하지 않습니다. 현재 5권 트레이에서는 manipulation이
        # 관측 x로 슬롯을 고른 뒤 그 슬롯의 실제 교정 치수를 사용합니다.
        grasp.thickness = self.default_book_thickness
        grasp.width = self.default_book_width
        grasp.height = self.default_book_height
        grasp.confidence = float(target["confidence"])

        result.success = True
        result.error_code = 0
        result.message = "Book grasp observation detected."

        self._publish_action_feedback(
            goal_handle,
            "TRANSFORMING_FRAME",
            len(detected_targets),
            grasp.confidence,
        )

        self._set_action_result(result)

    def _finish_action_with_failure(self, error_code, message):
        with self._action_lock:
            action_kind = self._active_kind

        if action_kind is None:
            return

        result = self._new_action_result(action_kind)
        result.success = False
        result.error_code = int(error_code)
        result.message = str(message)

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
        # 보조 confidence 이상인 책 검출 결과를 저장합니다. 기본/보조 후보
        # 선택은 Depth와 접근 가능한 ROI를 확인한 뒤 수행합니다.
        detections = []
        # 입력 RGB 이미지 한 장을 YOLO 모델로 추론합니다.
        results = self.model(
            rgb_image,
            verbose=False,
            conf=self.fallback_confidence_threshold,
        )
        # YOLO가 반환한 결과 묶음을 순회합니다.
        for result in results:
            # segmentation mask가 있으면 mask 텐서를 가져옵니다.
            masks = result.masks.data if result.masks is not None else None
            # 결과 안의 모든 bounding box를 순회합니다.
            for index, box in enumerate(result.boxes):
                # 현재 상자의 confidence를 실수형으로 변환합니다.
                confidence = float(box.conf[0])
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
    # 한 스레드는 action 결과를 기다리고, 나머지는 RGB/Depth/CameraInfo와
    # TF·서비스를 처리합니다. 센서 세 스트림이 action 대기를 굶기지 않도록
    # 최소 네 스레드를 둡니다.
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    # executor에 노드를 등록합니다.
    executor.add_node(node)

    try:
        # 노드가 종료될 때까지 ROS callback을 계속 실행합니다.
        executor.spin()

    except KeyboardInterrupt:
        # Ctrl+C 종료를 로그로 남깁니다.
        node.get_logger().warn("강제 종료")

    finally:
        # executor를 종료하고 노드를 제거합니다.
        executor.shutdown()
        node.destroy_node()

        # ROS가 아직 실행 중이면 통신을 종료합니다.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    # 이 파일을 직접 실행했을 때 main 함수를 호출합니다.
    main()
