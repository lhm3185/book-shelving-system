"""손목 RealSense(가정) + ROS2 발행 그래프 (Isaac Sim 5.1.0, 설치본 isaacsim.ros2.bridge 테스트 코드에서 노드 이름 확인).

발행: /rgb, /depth(32FC1, m), /camera_info, /tf (panda_link0 → wrist_camera), /clock (sim time)
프레임:
- 카메라 prim 이름 wrist_camera → TF 에 USD 카메라 축(-Z 앞, +Y 위)으로 나간다
- 이미지 frame_id 는 wrist_camera_optical_frame (+Z 앞, +Y 아래).
  wrist_camera → wrist_camera_optical_frame 은 **정적 TF 로 따로 붙인다** (X축 180°: qx=1).
  코드에서 축을 바꾸지 않는다 (웹 클로드 v7 회신 B절)

장착 가정 (레벨에 카메라가 없어 임시): panda_hand 원점에서 손 x 축으로 0.06 m, 광축 = 접근축(손 +Z),
가로 화각 90.5°(학습 데이터와 같음), 640×480. 확정되면 MOUNT_* 만 바꾼다.
"""
import math

import omni.graph.core as og
import usdrt.Sdf
from pxr import Gf, UsdGeom

MOUNT_OFFSET = (0.06, 0.0, 0.0)     # panda_hand 좌표계 (m)
HFOV_DEG = 90.5
RES = (640, 480)
CAMERA_NAME = "wrist_camera"
OPTICAL_FRAME = "wrist_camera_optical_frame"


def add_wrist_camera(stage, robot_path):
    path = f"{robot_path}/panda_hand/{CAMERA_NAME}"
    cam = UsdGeom.Camera.Define(stage, path)
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*MOUNT_OFFSET))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 0.0))     # USD 카메라 -Z → 손 +Z(접근축)
    aperture = 20.955
    cam.CreateHorizontalApertureAttr(aperture)
    cam.CreateVerticalApertureAttr(aperture * RES[1] / RES[0])
    cam.CreateFocalLengthAttr(aperture / (2 * math.tan(math.radians(HFOV_DEG) / 2)))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 50.0))
    return path


def build_ros_graph(camera_path, robot_path, rgb="/rgb", depth="/depth", info="/camera_info",
                    tf="/tf", clock="/clock", graph_path="/World/ArmRosGraph"):
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("Depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("Info", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("TF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(camera_path)]),
                ("RenderProduct.inputs:width", RES[0]),
                ("RenderProduct.inputs:height", RES[1]),
                ("RGB.inputs:type", "rgb"), ("RGB.inputs:topicName", rgb), ("RGB.inputs:frameId", OPTICAL_FRAME),
                ("Depth.inputs:type", "depth"), ("Depth.inputs:topicName", depth), ("Depth.inputs:frameId", OPTICAL_FRAME),
                ("Info.inputs:topicName", info), ("Info.inputs:frameId", OPTICAL_FRAME),
                ("TF.inputs:topicName", tf),
                # 로봇 루트를 넣으면 ridgeback_franka 트리가 world→panda_link2→…→base_link→…→world 로 **고리**가 된다
                # (2026-09-17 실측, tf2 "tree contains a loop"). 비전에 필요한 panda_link0→wrist_camera 만 낸다
                ("TF.inputs:targetPrims", [usdrt.Sdf.Path(camera_path)]),
                ("TF.inputs:parentPrim", [usdrt.Sdf.Path(f"{robot_path}/panda_link0")]),
                ("Clock.inputs:topicName", clock),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "RGB.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "Depth.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "Info.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "RGB.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath", "Depth.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath", "Info.inputs:renderProductPath"),
                ("Tick.outputs:tick", "TF.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "TF.inputs:timeStamp"),
                ("Tick.outputs:tick", "Clock.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "Clock.inputs:timeStamp"),
            ],
        },
    )
    return graph_path


def build_supplement_graph(camera_path, robot_path, tf="/tf", clock="/clock", graph_path="/World/ArmTfClockGraph"):
    """다른 담당자가 만든 카메라·ROS 그래프를 그대로 두고, 빠진 것만 보탠다 (franka_camera.usd, 2026-09-17).

    - panda_link0 → <카메라 prim 이름> TF (Isaac 이 광학 규약으로 발행 → 이미지 frame_id 와는 정적 항등 TF 로 잇는다)
    - /clock (sim time)
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("TF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("TF.inputs:topicName", tf),
                ("TF.inputs:targetPrims", [usdrt.Sdf.Path(camera_path)]),
                ("TF.inputs:parentPrim", [usdrt.Sdf.Path(f"{robot_path}/panda_link0")]),
                ("Clock.inputs:topicName", clock),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "TF.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "TF.inputs:timeStamp"),
                ("Tick.outputs:tick", "Clock.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "Clock.inputs:timeStamp"),
            ],
        },
    )
    return graph_path


def apply_amr_test_overrides(stage, robot_path, say=print):
    """franka_camera.usd(AMR 담당) 라이다·TF 그래프를 **시험 실행에서만** 보완한다. 파일은 수정하지 않는다.

    2026-09-17 라이다 점검 결과:
    - LaserScanPublish(type point_cloud)의 fullScan 이 False → 한 메시지가 30° 조각(76 Hz). SLAM 이 조각을 한 바퀴로 오인
    - TF·odom 노드 nodeNamespace '/World/ridgeback_franka' → /World/ridgeback_franka/tf, …/odom 으로 나가 표준 /tf, /odom 이 비어 있음
    """
    from pxr import Sdf
    g = f"{robot_path}/ros2_lidar_graph/LaserScanPublish"
    prim = stage.GetPrimAtPath(g)
    if prim.IsValid():
        a = prim.GetAttribute("inputs:fullScan") or prim.CreateAttribute("inputs:fullScan", Sdf.ValueTypeNames.Bool)
        a.Set(True)
        say(f"라이다 fullScan=True ({g})")
    for node in ("TFWorld2Odom", "TFOdom2Robot", "TFRobot", "PublisherOdometry"):
        prim = stage.GetPrimAtPath(f"{robot_path}/ros2_odom_graph/{node}")
        if prim.IsValid() and prim.GetAttribute("inputs:nodeNamespace"):
            prim.GetAttribute("inputs:nodeNamespace").Set("")
    say("TF·odom nodeNamespace 제거 → /tf, /odom")
    # TFRobot 이 로봇 전체(targetPrims=로봇 루트)를 내면 TF 고리 2개가 생긴다 (2026-09-17 실측):
    #   base_link→panda_link2→panda_link1→panda_link0→arm_mount_link→base_link
    #   base_link→dummy_base_y→dummy_base_x→world→odom→base_link (odom 그래프와 충돌)
    # → base_link 아래로 필요한 것만: panda_link0(팔·카메라 연결), Lidar(스캔)
    prim = stage.GetPrimAtPath(f"{robot_path}/ros2_odom_graph/TFRobot")
    if prim.IsValid():
        targets = [Sdf.Path(f"{robot_path}/panda_link0")]
        lidar = f"{robot_path}/front_laser/Lidar"
        if stage.GetPrimAtPath(lidar).IsValid():
            targets.append(Sdf.Path(lidar))
        prim.GetRelationship("inputs:targetPrims").SetTargets(targets)
        # USD 관계만 바꾸면 이미 불러온 OmniGraph 노드에 반영되지 않는다 (실측) → OmniGraph API 로도 설정
        try:
            og.Controller.set(og.Controller.attribute(f"{robot_path}/ros2_odom_graph/TFRobot.inputs:targetPrims"),
                              [usdrt.Sdf.Path(str(t)) for t in targets])
        except Exception as exc:     # noqa: BLE001
            say(f"TFRobot OmniGraph 설정 실패: {exc}")
        say(f"TFRobot 대상 → {[str(t).split('/')[-1] for t in targets]} (부모 base_link, 고리 제거)")


def build_lidar_tf_graph(lidar_path, robot_path, tf="/tf", graph_path="/World/AmrLidarTfGraph"):
    """base_link → <라이다 prim 이름> TF (라이다 frame_id 와 이름이 같아야 한다)"""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("TF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.SET_VALUES: [
                ("TF.inputs:topicName", tf),
                ("TF.inputs:targetPrims", [usdrt.Sdf.Path(lidar_path)]),
                ("TF.inputs:parentPrim", [usdrt.Sdf.Path(f"{robot_path}/base_link")]),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "TF.inputs:execIn"),
                ("SimTime.outputs:simulationTime", "TF.inputs:timeStamp"),
            ],
        },
    )
    return graph_path


class SensorGate:
    """카메라·라이다 발행 그래프를 작업 단계에 맞춰 켜고 끈다 (부하 절감, 2026-09-17).

    - 대기(트레이 관측)·AMR 이동: 켬 / 파지 확인 뒤 ~ 작업 끝: 끔
    - 카메라 발행 주기: 발행 노드 frameSkipCount, 헤드리스에서는 발행하는 스텝에만 렌더
    - 라이다가 켜져 있으면 한 바퀴 누적이 끊기지 않게 매 스텝 렌더
    노드 이름은 franka_camera.usd(AMR 담당)와 ros_sensors.build_ros_graph 기준. 없는 노드는 건너뛴다.
    """

    def __init__(self, stage, robot_path, say=print):
        self.say = say
        cams = [f"{robot_path}/Camera/{n}" for n in ("RenderProduct", "RGBPublish", "DepthPublish", "CameraInfoPublish")]
        cams += [f"/World/ArmRosGraph/{n}" for n in ("RenderProduct", "RGB", "Depth", "Info")]
        lidars = [f"{robot_path}/ros2_lidar_graph/{n}" for n in ("RenderProduct", "LaserScanPublish")]
        self.camera_nodes = [p for p in cams if stage.GetPrimAtPath(p).IsValid()]
        self.lidar_nodes = [p for p in lidars if stage.GetPrimAtPath(p).IsValid()]
        self.camera_on = bool(self.camera_nodes)
        self.lidar_on = bool(self.lidar_nodes)
        self.camera_every = 1

    def _set(self, paths, name, value):
        for p in paths:
            try:
                og.Controller.set(og.Controller.attribute(f"{p}.inputs:{name}"), value)
            except Exception as exc:     # noqa: BLE001
                self.say(f"센서 설정 실패 {p}.{name}: {exc}")

    @staticmethod
    def camera_skip(hz, sim_hz=60):
        return max(0, int(round(sim_hz / max(hz, 1e-3))) - 1)

    @staticmethod
    def preset_camera_hz(stage, robot_path, hz, say=print, sim_hz=60):
        """재생(초기화) 전에 USD 로 넣어야 반영된다. 실행 중 OmniGraph 로 바꾸면 무시됨 (2026-09-17 실측: 78 Hz 유지)"""
        from pxr import Sdf
        skip = SensorGate.camera_skip(hz, sim_hz)
        for n in ("RGBPublish", "DepthPublish", "CameraInfoPublish"):
            prim = stage.GetPrimAtPath(f"{robot_path}/Camera/{n}")
            if prim.IsValid():
                a = prim.GetAttribute("inputs:frameSkipCount") or prim.CreateAttribute("inputs:frameSkipCount", Sdf.ValueTypeNames.UInt)
                a.Set(skip)
        say(f"카메라 발행 {sim_hz / (skip + 1):.0f} Hz (frameSkipCount {skip})")
        return skip

    def set_camera_hz(self, hz, sim_hz=60):
        """렌더 간격만 맞춘다 (발행 주기 자체는 preset_camera_hz 로 재생 전에)"""
        self.camera_every = self.camera_skip(hz, sim_hz) + 1

    def camera(self, on):
        if self.camera_nodes and on != self.camera_on:
            self._set(self.camera_nodes, "enabled", on)
            self.camera_on = on

    def lidar(self, on):
        if self.lidar_nodes and on != self.lidar_on:
            self._set(self.lidar_nodes, "enabled", on)
            self.lidar_on = on

    def all(self, on, why=""):
        before = (self.camera_on, self.lidar_on)
        self.camera(on)
        self.lidar(on)
        if before != (self.camera_on, self.lidar_on):
            self.say(f"센서 {'켬' if on else '끔'} (카메라 {self.camera_on}, 라이다 {self.lidar_on}) {why}")

    def need_render(self, step, gui=False):
        if gui or self.lidar_on:
            return True
        return self.camera_on and step % self.camera_every == 0
