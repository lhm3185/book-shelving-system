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
from shelving_interfaces.action import DetectTargetSlot
# 향후 선반 슬롯 메시지에 사용할 타입입니다.
from shelving_interfaces.msg import TargetSlot
# ROS 2 노드의 기본 클래스입니다.
from rclpy.node import Node
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
# RGB/Depth 영상과 카메라 내부 파라미터 메시지입니다.
from sensor_msgs.msg import Image, CameraInfo
# 책 위치와 책 pose를 발행할 메시지입니다.
from geometry_msgs.msg import PointStamped, PoseStamped
# 수동 검출 요청 토픽의 Boolean 메시지입니다.
from std_msgs.msg import Bool
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
        # 검출 상자와 confidence를 그린 디버그 영상을 발행할 토픽입니다.
        self.debug_image_topic = self.declare_parameter(
            'debug_image_topic', '/perception/debug_image').value
        # 목표점 투영과 좌/중/우 Depth 샘플을 그린 전용 디버그 영상입니다.
        self.depth_debug_image_topic = self.declare_parameter(
            'depth_debug_image_topic',
            '/perception/depth_debug_image',
        ).value
        # Depth 디버그 영상을 흰색으로 포화시킬 최대 거리(m)입니다.
        self.depth_debug_max_m = float(self.declare_parameter(
            'depth_debug_max_m', 3.0).value)
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
        # 중심(C)과 좌/우(L/R) Depth 샘플 창의 반경과 간격입니다.
        self.target_sample_radius_px = int(self.declare_parameter(
            'target_sample_radius_px', 5).value)
        self.target_side_offset_px = int(self.declare_parameter(
            'target_side_offset_px', 25).value)
        # 중심 Depth가 좌우보다 이 값 이상 멀면 경계가 있다고 판정합니다.
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
        if self.depth_debug_max_m <= 0:
            raise ValueError('depth_debug_max_m must be positive')

        # 학습된 YOLO 모델을 메모리에 로드합니다.
        self.model = YOLO(self.model_path)
        # 책장 segmentation YOLO 모델을 별도로 로드합니다.
        self.shelf_model = YOLO(self.shelf_model_path)
        # 검출 상자와 깊이로 3차원 좌표를 계산하는 객체를 생성합니다.
        self.book_detector = BookDetector()
        # 단일 월드 목표점의 좌/중/우 Depth 차이를 측정하는 객체입니다.
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
        # action 상태를 여러 스레드에서 안전하게 접근하기 위한 lock입니다.
        self._action_lock = threading.Lock()
        # 센서 처리 완료를 action 실행 콜백에 알려주는 이벤트입니다.
        self._action_event = threading.Event()
        # 현재 실행 중인 action goal을 저장합니다.
        self._active_goal = None
        # 센서 처리 결과를 action 콜백에 전달하기 위한 변수입니다.
        self._action_result = None
        # 책을 잡은 뒤 사용할 빈 위치를 검출 프레임 사이에 보존합니다.
        self._remembered_empty_position = None
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
        # 검출 결과를 그린 디버그 영상을 Image 메시지로 발행합니다.
        self.debug_image_pub = self.create_publisher(
            Image, self.debug_image_topic, 10)
        # Depth 자체 위에서 목표점과 세 샘플 범위를 확인할 디버그 영상입니다.
        self.depth_debug_image_pub = self.create_publisher(
            Image, self.depth_debug_image_topic, 10)
        # 찾은 빈 단마다 카메라/로봇 기준 3D 점을 발행합니다.
        self.empty_slot_pub = self.create_publisher(
            PointStamped, self.empty_slot_topic, 10)
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
            rgb_image = _imgmsg_to_bgr(rgb_msg)
            # Depth 메시지는 원래 숫자 encoding을 유지한 채 변환합니다.
            depth_image = _imgmsg_to_array(depth_msg)
        except ValueError as error:
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
            # **트레이 ROI 밖의 책은 버립니다.** 서가에 꽂힌 책·배경이 섞이면
            # 1등 선택이 엉뚱한 책을 고릅니다. 책 경로에만 적용합니다 —
            # 책장 빈 공간 판정에는 ROI 를 걸지 않습니다.
            detected_targets, roi_dropped = self._filter_book_roi(
                detected_targets, rgb_msg.header.frame_id, rgb_msg.header.stamp)
            # 책장 전체의 bbox와 클래스를 책장 YOLO 모델로 검출합니다.
            shelf_detection = self._detect_shelf(rgb_image)
            # 책장 ROI 안에서 좌우보다 깊은 연결영역을 찾아 빈 공간의 중심과
            # 좌/중/우 Depth를 얻습니다. 고정 index/월드 좌표는 사용하지 않습니다.
            target_inspection = self.target_detector.find_empty_position(
                depth_image,
                shelf_detection['box'] if shelf_detection else None,
                fx,
                fy,
                cx,
                cy,
                depth_scale,
            )
            # 책과 책장 검출 결과를 화면과 debug image 토픽에 표시합니다.
            self._publish_debug_image(
                rgb_image,
                rgb_msg.header,
                detections,
                shelf_detection,
                target_inspection,
                roi_px=self._book_roi_pixels(
                    rgb_msg.header.frame_id, rgb_msg.header.stamp,
                    fx, fy, cx, cy, rgb_image.shape),
                keep_boxes=[t['box'] for t in detected_targets],
                dropped=roi_dropped,
            )
            self._publish_depth_debug_image(
                depth_image,
                depth_scale,
                depth_msg.header,
                target_inspection,
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

        # **신뢰도가 가장 높은 책 한 권만** 로봇팔에 보냅니다.
        # 토픽은 한 프레임에 여러 점을 따로 쏘면 받는 쪽이 "어느 것이 1등인지",
        # "어디까지가 같은 프레임인지"를 알 수 없습니다. 그래서 하나만 냅니다.
        # 전부 보고 싶을 때는 publish_best_only:=false (디버그용).
        if detected_targets and self.publish_best_only:
            best = max(detected_targets, key=lambda t: t['confidence'])
            publish_targets = [best]
        else:
            publish_targets = detected_targets
        for target in publish_targets:
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
                f"Book picked(best): xyz=({book_point.point.x:.3f}, "
                f"{book_point.point.y:.3f}, {book_point.point.z:.3f}), "
                f"center={target['center']}, conf={target['confidence']:.2f}, "
                f"depth={target.get('depth_source', '?')}, "
                f"yaw={target['image_angle']:.3f} rad "
                f"(후보 {len(detected_targets)}권)")

        # 투영 및 좌/우 깊이 경계가 모두 확인된 경우에만 목표점을 빈 위치로 사용합니다.
        transformed_empty_position = None
        if target_inspection is not None:
            self._log_target_inspection(target_inspection)
        if target_inspection and target_inspection['is_empty']:
            transformed_empty_position = self._transform_xyz(
                target_inspection['target_xyz'],
                rgb_msg.header.frame_id,
                rgb_msg.header.stamp,
            )
        if transformed_empty_position is not None:
            empty_msg = PointStamped()
            empty_msg.header = transformed_empty_position.header
            empty_msg.point = transformed_empty_position.point
            # 책이 없는 책장 스캔에서는 검출 즉시 슬롯을 발행합니다.
            # 책도 검출된 프레임에서는 아래에서 책 처리 후 다시 발행합니다.
            if not detected_targets:
                self.empty_slot_pub.publish(empty_msg)
            self.get_logger().info(
                f'Empty shelf target: '
                f'xyz=({transformed_empty_position.point.x:.3f}, '
                f'{transformed_empty_position.point.y:.3f}, '
                f'{transformed_empty_position.point.z:.3f})')

        # 현재 스캔에서 빈 위치를 확인했을 때만 캐시를 갱신합니다.
        # 다음 책 검출 프레임에서 깊이를 다시 읽지 못해도 이 좌표를 사용합니다.
        if transformed_empty_position is not None:
            self._remembered_empty_position = transformed_empty_position

        # 현재 프레임의 결과를 우선하고, 없으면 이전 검출에서 기억한 좌표를 사용합니다.
        placement_position = (
            transformed_empty_position or self._remembered_empty_position)

        # 책 검출 후에도 기억한 삽입 좌표를 다시 발행해 로봇 팔이 받도록 합니다.
        if detected_targets and placement_position is not None:
            self.empty_slot_pub.publish(placement_position)
            self.get_logger().info(
                'Resent remembered empty shelf position after book detection')

        # action 요청이었다면 기억된 단일 빈 위치를 반환합니다.
        if active_goal is not None:
            self._complete_action_from_empty_position(
                active_goal,
                placement_position,
            )

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
                self.target_frame,
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
                f'{self.target_frame}')
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
                    pt.header.frame_id = self.target_frame
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
        target_inspection=None,
        roi_px=None,
        keep_boxes=None,
        dropped=0,
    ):
        """책장·책·월드 목표점 표시를 그린 RGB 영상을 발행합니다."""
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
        self._draw_target_inspection(debug_image, target_inspection)
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

    def _publish_depth_debug_image(
        self,
        depth_image,
        depth_scale,
        header,
        target_inspection,
    ):
        """미터 Depth를 회색조로 바꾸고 목표점 및 L/C/R 측정을 표시합니다."""
        try:
            depth_m = depth_image.astype(np.float32) * float(depth_scale)
            valid = np.isfinite(depth_m) & (depth_m > 0.0)
            gray = np.zeros(depth_m.shape, dtype=np.uint8)
            gray[valid] = np.clip(
                depth_m[valid] / self.depth_debug_max_m * 255.0,
                0.0,
                255.0,
            ).astype(np.uint8)
            debug_image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            self._draw_target_inspection(debug_image, target_inspection)
            debug_msg = _bgr8_to_imgmsg(debug_image)
            debug_msg.header = header
            self.depth_debug_image_pub.publish(debug_msg)
        except Exception as error:       # noqa: BLE001 - 화면 때문에 노드가 죽으면 안 됩니다
            self.get_logger().warning(
                f'Depth 디버그 영상 표시 실패(검출은 계속합니다): '
                f'{type(error).__name__}: {error}',
                throttle_duration_sec=10.0,
            )

    @staticmethod
    def _draw_target_inspection(image, inspection):
        """목표 픽셀, L/C/R 샘플 창, 깊이값과 판정 결과를 그립니다."""
        if not inspection or inspection.get('pixel') is None:
            return

        u, v = inspection['pixel']
        height, width = image.shape[:2]
        if not (0 <= u < width and 0 <= v < height):
            return

        colors = {
            'left': (255, 128, 0),
            'center': (0, 0, 255),
            'right': (0, 255, 255),
        }
        names = {'left': 'L', 'center': 'C', 'right': 'R'}
        for name, bounds in inspection.get('sample_windows', {}).items():
            x1, y1, x2, y2 = bounds
            color = colors[name]
            cv2.rectangle(image, (x1, y1), (x2 - 1, y2 - 1), color, 2)
            depth = inspection.get(f'{name}_depth')
            depth_text = 'n/a' if depth is None else f'{depth:.3f}m'
            cv2.putText(
                image,
                f'{names[name]} {depth_text}',
                (max(0, x1 - 2), max(16, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )

        # 사용자가 그림판에서 표시한 것과 같은 빨간 중심점을 항상 찍습니다.
        cv2.circle(image, (u, v), 7, (0, 0, 255), -1)
        passed = bool(inspection.get('is_empty'))
        status_color = (0, 255, 0) if passed else (0, 128, 255)
        left_diff = inspection.get('left_difference')
        right_diff = inspection.get('right_difference')
        left_text = 'n/a' if left_diff is None else f'{left_diff:+.3f}'
        right_text = 'n/a' if right_diff is None else f'{right_diff:+.3f}'
        label_y = min(height - 8, v + 35)
        cv2.putText(
            image,
            f'dL={left_text}m dR={right_text}m '
            f'{"PASS" if passed else "CHECK"}',
            (max(0, min(width - 280, u - 120)), label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            status_color,
            2,
            cv2.LINE_AA,
        )

    def _log_target_inspection(self, inspection):
        """한 프레임의 목표점 투영 및 Depth 대비를 사람이 읽기 쉽게 기록합니다."""
        pixel = inspection.get('pixel')
        camera_xyz = inspection['camera_xyz']
        if pixel is None or not inspection.get('visible'):
            self.get_logger().warning(
                f'Target projection is outside the depth image: pixel={pixel}, '
                f'camera_xyz=({camera_xyz[0]:.3f}, {camera_xyz[1]:.3f}, '
                f'{camera_xyz[2]:.3f})')
            return

        def format_value(value):
            return 'n/a' if value is None else f'{value:.3f}'

        self.get_logger().info(
            f'Empty center pixel={pixel}, shelf_front_z={camera_xyz[2]:.3f}m; '
            f'Depth L/C/R='
            f'{format_value(inspection["left_depth"])}/'
            f'{format_value(inspection["center_depth"])}/'
            f'{format_value(inspection["right_depth"])}m; '
            f'center-left={format_value(inspection["left_difference"])}m, '
            f'center-right={format_value(inspection["right_difference"])}m; '
            f'contrast={"PASS" if inspection["is_empty"] else "CHECK"}')

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

    def _complete_action_from_empty_position(self, goal_handle, empty_point):
        """검증된 단일 빈 책장 위치를 DetectTargetSlot 결과로 반환합니다."""
        # action 결과 메시지를 생성합니다.
        result = DetectTargetSlot.Result()
        result.candidate_count = int(empty_point is not None)
        # 좌우 Depth 경계를 확인하지 못했으면 action을 실패시킵니다.
        if empty_point is None:
            result.error_code = 3003
            result.message = 'No empty shelf position was detected.'
            self._set_action_result(result)
            return

        self._publish_action_feedback(
            goal_handle,
            'CALCULATING_POSE',
            1,
            1.0,
        )

        # action 결과의 TargetSlot에 빈 단 위치를 기록합니다.
        slot = result.target_slot
        slot.header = empty_point.header
        slot.pose.position.x = empty_point.point.x
        slot.pose.position.y = empty_point.point.y
        slot.pose.position.z = empty_point.point.z
        # 로봇팔 삽입 규약인 yaw +90도 회전을 quaternion으로 설정합니다.
        slot.pose.orientation.x = 0.0
        slot.pose.orientation.y = 0.0
        slot.pose.orientation.z = math.sqrt(0.5)
        slot.pose.orientation.w = math.sqrt(0.5)

        # goal의 책 크기를 이용해 삽입 가능한 폭과 높이를 계산합니다.
        goal = goal_handle.request
        slot.available_width = max(
            0.0,
            float(goal.book_thickness + 2.0 * goal.safety_margin),
        )
        slot.available_height = max(
            0.0,
            float(goal.book_height + goal.safety_margin),
        )
        # 삽입 관련 파라미터를 결과에 기록합니다.
        slot.insertion_depth = self.insertion_depth
        slot.pre_insert_offset = self.pre_insert_offset
        # 깊이 기반 빈 공간 판정의 기본 confidence를 기록합니다.
        slot.confidence = 1.0
        result.success = True
        result.error_code = 0
        result.message = 'Empty shelf position detected from depth.'
        self._publish_action_feedback(
            goal_handle,
            'TRANSFORMING_FRAME',
            1,
            slot.confidence,
        )
        # execute callback이 결과를 받아 action을 끝내도록 저장합니다.
        self._set_action_result(result)

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
        # executor를 종료하고 노드를 제거합니다.
        executor.shutdown()
        node.destroy_node()

        # ROS가 아직 실행 중이면 통신을 종료합니다.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    # 이 파일을 직접 실행했을 때 main 함수를 호출합니다.
    main()
