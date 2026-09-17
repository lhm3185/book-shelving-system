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
