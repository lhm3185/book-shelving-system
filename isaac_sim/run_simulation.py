#!/usr/bin/env python3
"""Run one configured Isaac Sim navigation scene.

A temporary root layer composes the environment and robot before Isaac Sim
opens the stage. This avoids reopening an active OmniGraph stage when the
robot layer is added. Nothing is saved into the environment or robot USD.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import signal
import tempfile

import yaml


# All runtime inputs live below isaac_sim/.  Keeping this root local prevents
# the simulator from depending on Desktop copies or other machine-specific
# working directories.
PROJECT_ROOT = Path(__file__).resolve().parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="library_map")
    parser.add_argument(
        "--config",
        default=str(PROJECT_ROOT / "config" / "scenes.yaml"),
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--enable-manipulation",
        action="store_true",
        help="Enable the production camera and Franka manipulation executor.",
    )
    return parser.parse_args()


def resolve_project_path(value: str) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path.resolve()


def load_scene(config_path: Path, scene_name: str) -> dict:
    if not config_path.is_file():
        raise FileNotFoundError(f"Scene config not found: {config_path}")

    with config_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}

    scenes = document.get("scenes", {})
    if scene_name not in scenes:
        available = ", ".join(sorted(scenes)) or "<none>"
        raise KeyError(
            f"Unknown scene '{scene_name}'. Available scenes: {available}"
        )

    scene = scenes[scene_name]
    required = {
        "world_usd": scene.get("world_usd"),
        "tray_usd": scene.get("tray_usd"),
        "nav_map_yaml": scene.get("nav_map_yaml"),
        "robot.usd": scene.get("robot", {}).get("usd"),
        "robot.prim_path": scene.get("robot", {}).get("prim_path"),
    }
    missing = [name for name, value in required.items() if not value]
    if missing:
        raise ValueError(f"Missing scene fields: {', '.join(missing)}")

    for label in ("world_usd", "tray_usd", "nav_map_yaml"):
        path = resolve_project_path(scene[label])
        if not path.is_file():
            raise FileNotFoundError(f"{label} not found: {path}")

    robot_path = resolve_project_path(scene["robot"]["usd"])
    if not robot_path.is_file():
        raise FileNotFoundError(f"robot.usd not found: {robot_path}")

    return scene


ARGS = parse_args()
CONFIG_PATH = Path(ARGS.config).expanduser().resolve()
SCENE = load_scene(CONFIG_PATH, ARGS.scene)

WORLD_USD = resolve_project_path(SCENE["world_usd"])
TRAY_USD = resolve_project_path(SCENE["tray_usd"])
ROBOT_USD = resolve_project_path(SCENE["robot"]["usd"])
ROBOT_PRIM_PATH = SCENE["robot"]["prim_path"]
ROS_DOMAIN_ID = int(SCENE.get("ros", {}).get("domain_id", 0))

RUNTIME_CONFIG = SCENE.get("runtime", {})

TRAY_CONFIG = SCENE.get("tray", {})
MANIPULATION_CONFIG = SCENE.get("manipulation", {})

if not TRAY_CONFIG:
    raise ValueError(
        "Scene configuration must contain a tray section"
    )

SCENARIO_STATE_TOPIC = str(
    RUNTIME_CONFIG.get(
        "scenario_state_topic",
        "/simulation/scenario/state",
    )
)

REQUIRED_GRAPH_PATHS = tuple(
    str(graph_path)
    for graph_path in RUNTIME_CONFIG.get(
        "required_graphs",
        (),
    )
)

if not REQUIRED_GRAPH_PATHS:
    raise ValueError(
        "runtime.required_graphs must contain "
        "at least one Action Graph path"
    )

def create_composed_stage() -> Path:
    pose = SCENE["robot"].get("simulation_pose", {})
    position = pose.get("position", [0.0, 0.0, 0.0])
    if len(position) != 3:
        raise ValueError("robot.simulation_pose.position must contain x, y, z")

    x, y, z = (float(value) for value in position)
    yaw = float(pose.get("yaw", 0.0))
    yaw_degrees = math.degrees(yaw)

    # Put the robot layer first so its Action Graph and robot overrides are the
    # stronger opinions. Both asset paths are absolute only inside this
    # disposable runtime layer; the project configuration stays portable.
    document = f'''#usda 1.0
(
    defaultPrim = "World"
    startTimeCode = 0
    endTimeCode = 1000000
    timeCodesPerSecond = 60
    framesPerSecond = 60
    metersPerUnit = 1
    upAxis = "Z"
    subLayers = [
        @{ROBOT_USD.as_posix()}@,
        @{TRAY_USD.as_posix()}@,
        @{WORLD_USD.as_posix()}@
    ]
)

over "World"
{{
    over "ridgeback_franka"
    {{
        double3 xformOp:translate = ({x:.17g}, {y:.17g}, {z:.17g})
        # Keep the articulation root aligned with the world axes. The mobile
        # base uses world-X/world-Y prismatic joints, and its controller already
        # converts body-frame Twist commands into those world axes. Rotating the
        # root as well would rotate the commanded velocity twice.
        quatd xformOp:orient = (1, 0, 0, 0)

        over "dummy_base_y"
        {{
            over "dummy_base_revolute_z_joint"
            {{
                float state:angular:physics:position = {yaw_degrees:.17g}
                float drive:angular:physics:targetPosition = {yaw_degrees:.17g}
            }}
        }}
    }}

    def PhysicsScene "PhysicsScene"
    {{
        vector3f physics:gravityDirection = (0, 0, -1)
        float physics:gravityMagnitude = 9.81
    }}
}}

over "Graphs"
{{
    over "Nav2BaseController"
    {{
        over "ReadSimulationTime"
        {{
            custom bool inputs:resetOnStop = 0
        }}
    }}

    over "LidarSensorGraph"
    {{
        over "ReadSimulationTime"
        {{
            custom bool inputs:resetOnStop = 0
        }}

        over "PublishLidarPointCloud"
        {{
            custom bool inputs:resetSimulationTimeOnStop = 0
        }}
    }}
}}
'''
    handle = tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        prefix="ridgeback_franka_",
        suffix=".usda",
        delete=False,
    )
    with handle:
        handle.write(document)
    return Path(handle.name)


COMPOSED_STAGE = create_composed_stage()

# The ROS bridge reads these while Isaac Sim starts.
os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

# SimulationApp must be created before importing Omniverse/Isaac modules.
from isaacsim.simulation_app import SimulationApp


simulation_app = SimulationApp({"headless": ARGS.headless})

import omni.kit.app
import omni.timeline
import omni.usd
import omni.graph.core as og
import carb
import numpy as np
from pxr import UsdGeom
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils import stage as stage_utils
from isaacsim.core.utils.types import ArticulationAction

from lib import ros_bridge
from lib.scenario_runtime import ScenarioRuntime
from lib.tray_runtime import TrayRuntime
from lib.sensors.camera_bridge import (
    OPTICAL_FRAME,
    add_wrist_camera,
    build_mobile_base_to_arm_tf_graph,
    build_ros_graph,
    embedded_camera_local_translation,
)

if ARGS.enable_manipulation:
    os.environ["SIM_SHELF_PRIM"] = str(
        MANIPULATION_CONFIG.get(
            "shelf_prim_path",
            "/World/bookshelves/shelf_brown__book_shelf_01",
        )
    )
    os.environ["SIM_SHELF_ROW_Z"] = str(
        MANIPULATION_CONFIG.get("shelf_row_z", 0.497)
    )
    sweep_boards = (
        MANIPULATION_CONFIG.get(
            "sweep_boards_world",
            [1.042, 0.498],
        )
    )

    os.environ["SIM_SWEEP_BOARDS"] = (
        ",".join(
            str(float(value))
            for value in sweep_boards
        )
    )

    os.environ["SIM_SWEEP_X_FROM"] = str(
        MANIPULATION_CONFIG.get("sweep_x_from", -0.35)
    )
    os.environ["SIM_SWEEP_X_TO"] = str(
        MANIPULATION_CONFIG.get("sweep_x_to", 0.35)
    )
    os.environ["SIM_SWEEP_POINTS"] = str(
        MANIPULATION_CONFIG.get("sweep_points", 5)
    )
    os.environ["ARM_ROBOT"] = "franka"
    controller_path = PROJECT_ROOT / "lib" / "controllers"
    manipulation_package_path = (
        PROJECT_ROOT.parent
        / "ros2_ws"
        / "src"
        / "shelving_manipulation"
    )
    import sys
    sys.path.insert(0, str(controller_path))
    sys.path.insert(0, str(manipulation_package_path))
    from book_scene import BookScene
    from manipulation_executor import ManipulationExecutor


stop_requested = False
scenario_node = None
scenario_runtime = None
tray_runtime = None
manipulation_scene = None
manipulation_executor = None
world = None
timeline = None


class RuntimeWorld:
    """기존 SimulationApp의 물리 루프를 manipulation API에 맞춰 노출한다.

    별도 Isaac World를 만들면 USD 안의 손목 카메라 rigid body까지 다시
    tensor scene에 등록하려 하므로, 이미 검증된 standalone 소유권을 유지한다.
    """

    def __init__(self, app, physics_dt=1 / 60):
        self._app = app
        self._physics_dt = float(physics_dt)
        self._callbacks = {}

    def step(self, render=True):
        del render  # SimulationApp이 GUI/headless 설정에 맞춰 렌더링한다.
        self._app.update()
        for callback in tuple(self._callbacks.values()):
            callback(self._physics_dt)

    def get_physics_dt(self):
        return self._physics_dt

    def add_physics_callback(self, name, callback):
        if name in self._callbacks:
            raise ValueError(f"duplicate physics callback: {name}")
        self._callbacks[name] = callback


def request_stop(_signum, _frame) -> None:
    global stop_requested
    stop_requested = True


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)


def update_until_stage_loaded() -> None:
    while stage_utils.is_stage_loading():
        simulation_app.update()
    for _ in range(10):
        simulation_app.update()
    # In standalone/headless mode the ROS bridge may otherwise defer creating
    # publishers until it verifies a subscriber. The runner discovers topics
    # before Nav2 starts, so publishers must be created unconditionally.
    carb.settings.get_settings().set_bool(
        "/exts/isaacsim.ros2.bridge/publish_without_verification", True
    )

def ensure_required_graphs(stage) -> None:
    """필수 Action Graph를 검사하고 활성화한다."""
    for graph_path in REQUIRED_GRAPH_PATHS:
        graph_prim = stage.GetPrimAtPath(graph_path)

        if not graph_prim.IsValid():
            raise RuntimeError(
                f"Required graph is missing: {graph_path}"
            )

        graph = og.get_graph_by_path(graph_path)

        if not graph.is_valid():
            raise RuntimeError(
                "OmniGraph runtime object is invalid: "
                f"{graph_path}"
            )

        was_disabled = graph.is_disabled()

        if was_disabled:
            graph.set_disabled(False)

        if graph.is_disabled():
            raise RuntimeError(
                f"Failed to enable graph: {graph_path}"
            )

        print(
            f"[project] graph={graph_path} "
            f"was_disabled={was_disabled} "
            f"disabled={graph.is_disabled()}",
            flush=True,
        )

try:
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    extension_manager.set_extension_enabled_immediate(
        "isaacsim.ros2.bridge", True
    )
    # Extension registration is asynchronous. Opening a stage containing ROS
    # OmniGraph nodes in the same frame can race node-type registration and
    # crash Isaac Sim 5.1 inside omni.graph/omni.usd.
    for _ in range(10):
        simulation_app.update()

    print(f"[project] scene={ARGS.scene}")
    print(f"[project] world={WORLD_USD}", flush=True)
    print(f"[project] tray_layer={TRAY_USD}", flush=True)
    print(f"[project] robot_layer={ROBOT_USD}", flush=True)
    print(f"[project] composed_stage={COMPOSED_STAGE}", flush=True)
    print(f"[project] ROS_DOMAIN_ID={ROS_DOMAIN_ID}")

    if not stage_utils.open_stage(str(COMPOSED_STAGE)):
        raise RuntimeError(f"Failed to open composed stage: {COMPOSED_STAGE}")
    update_until_stage_loaded()

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Isaac Sim did not provide an opened USD stage")

    robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if not robot_prim.IsValid():
        raise RuntimeError(
            f"Robot prim was not composed at {ROBOT_PRIM_PATH}. "
            "Check the robot USD layer."
        )

    ensure_required_graphs(stage)

    if ARGS.enable_manipulation:
        embedded_camera_prim_path = str(
            MANIPULATION_CONFIG.get("camera_prim_path", "")
        )
        if not embedded_camera_prim_path:
            raise ValueError(
                "manipulation.camera_prim_path is required in full mode"
            )
        if not stage.GetPrimAtPath(embedded_camera_prim_path).IsValid():
            raise RuntimeError(
                "Embedded manipulation camera prim is missing: "
                f"{embedded_camera_prim_path}"
            )

        embedded_camera_translation = embedded_camera_local_translation(
            stage,
            ROBOT_PRIM_PATH,
            embedded_camera_prim_path,
        )

        # 에셋의 RSD455에는 panda_hand 아래에 중첩된 rigid body가 있다.
        # PhysX 오류를 막기 위해 해당 body만 독립 xform stack으로 두되, 이
        # 카메라는 영상 소스로 사용하지 않는다. 부모 변환이 끊겨 검은 화면이
        # 되기 때문이다. 팀원 R&D 코드와 같이 물리 body가 없는 경량 카메라를
        # panda_hand에 별도로 달아 manipulation 영상만 발행한다.
        embedded_camera_body_path = embedded_camera_prim_path.rsplit("/", 1)[0]
        embedded_camera_body_prim = stage.GetPrimAtPath(
            embedded_camera_body_path
        )
        if embedded_camera_body_prim.IsValid():
            UsdGeom.Xformable(
                embedded_camera_body_prim
            ).SetResetXformStack(True)
            print(
                "[project] isolated embedded camera rigid body: "
                f"{embedded_camera_body_path}",
                flush=True,
            )

        camera_prim_path = add_wrist_camera(
            stage,
            ROBOT_PRIM_PATH,
            mount_offset=embedded_camera_translation,
        )
        build_ros_graph(
            camera_prim_path,
            ROBOT_PRIM_PATH,
            graph_path="/Graphs/ArmCameraGraph",
            frame_id=OPTICAL_FRAME,
            publish_clock=False,
        )

        arm_base_tf_graph = (
            build_mobile_base_to_arm_tf_graph(
                ROBOT_PRIM_PATH,
                graph_path="/Graphs/ArmBaseTfGraph",
            )
        )

        print(
            "[project] mobile-base TF bridge="
            f"{arm_base_tf_graph} "
            "base_link -> panda_link0",
            flush=True,
        )

        print(
            f"[project] manipulation_camera={camera_prim_path} "
            f"frame_id={OPTICAL_FRAME}",
            flush=True,
        )

    physics_scene_path = "/World/PhysicsScene"
    if not stage.GetPrimAtPath(physics_scene_path).IsValid():
        raise RuntimeError(
            f"Required PhysicsScene is missing: "
            f"{physics_scene_path}"
        )
    print(f"[project] physics_scene={physics_scene_path}")

    timeline = omni.timeline.get_timeline_interface()

    scenario_node = ros_bridge.make_node(
        "isaac_scenario_runtime"
    )

    scenario_runtime = ScenarioRuntime(
        node=scenario_node,
        timeline=timeline,
        state_topic=SCENARIO_STATE_TOPIC,
    )

    scenario_runtime.add_reset_hook(
        "required_action_graphs",
        lambda: ensure_required_graphs(stage),
    )

    # 먼저 Timeline을 재생해야 PhysX articulation handle을 만들 수 있다.
    scenario_runtime.start()
    simulation_app.update()

    world = RuntimeWorld(simulation_app)

    robot = SingleArticulation(
        prim_path=ROBOT_PRIM_PATH,
        name="scenario_robot",
    )
    robot.initialize()

    required_robot_joints = {
        "dummy_base_prismatic_x_joint",
        "dummy_base_prismatic_y_joint",
        "dummy_base_revolute_z_joint",
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
        "panda_finger_joint1",
        "panda_finger_joint2",
    }

    missing_robot_joints = (
        required_robot_joints - set(robot.dof_names)
    )
    if missing_robot_joints:
        raise RuntimeError(
            "Robot articulation is missing required joints: "
            + ", ".join(sorted(missing_robot_joints))
        )

    initial_robot_joint_positions = np.array(
        robot.get_joint_positions(),
        dtype=float,
        copy=True,
    )

    if initial_robot_joint_positions.size != robot.num_dof:
        raise RuntimeError(
            "Failed to capture the complete robot articulation state"
        )

    initial_robot_joint_velocities = np.zeros_like(
        initial_robot_joint_positions
    )

    print(
        "[project] captured robot reset state: "
        f"dof_count={robot.num_dof}",
        flush=True,
    )

    def reset_robot_articulation() -> None:
        robot.initialize()

        robot.apply_action(
            ArticulationAction(
                joint_positions=initial_robot_joint_positions,
                joint_velocities=initial_robot_joint_velocities,
            )
        )

        robot.set_joint_positions(
            initial_robot_joint_positions
        )
        robot.set_joint_velocities(
            initial_robot_joint_velocities
        )

        print(
            "[project] robot articulation restored",
            flush=True,
        )

    tray_runtime = TrayRuntime(
        node=scenario_node,
        stage=stage,
        timeline=timeline,
        config=TRAY_CONFIG,
    )

    if ARGS.enable_manipulation:
        manipulation_scene = BookScene(
            app=simulation_app,
            usd=str(COMPOSED_STAGE),
            tray_usd=str(TRAY_USD),
            tray_center=TRAY_CONFIG["handoff_position_base"][:2],
            n_books=len(tray_runtime.book_paths),
            place_dx=(),
            say=lambda message: print(
                f"[manipulation] {message}",
                flush=True,
            ),
            stage=stage,
            world=world,
            robot=robot,
            tray_path=tray_runtime.tray_path,
            book_paths=tray_runtime.book_paths,
        )
        manipulation_executor = ManipulationExecutor(
            scene=manipulation_scene,
            node=scenario_node,
            say=lambda message: print(
                f"[manipulation] {message}",
                flush=True,
            ),
            command_topic=str(
                MANIPULATION_CONFIG.get(
                    "command_topic",
                    "/manipulation/sim/command",
                )
            ),
            state_topic=str(
                MANIPULATION_CONFIG.get(
                    "state_topic",
                    "/manipulation/sim/state",
                )
            ),
            start_home="move",
            render=True,
            gui=not ARGS.headless,
            tray_runtime=tray_runtime,
        )
        print(
            "[project] production manipulation executor enabled",
            flush=True,
        )

    # FixedJoint를 먼저 제거한다.
    scenario_runtime.add_reset_hook(
        "tray_runtime",
        tray_runtime.reset,
    )

    # 그다음 로봇을 초기 위치로 복원한다.
    scenario_runtime.add_reset_hook(
        "robot_articulation",
        reset_robot_articulation,
    )

    # 모든 초기 상태와 reset hook이 준비된 후 READY를 발행한다.
    scenario_runtime.update()
    tray_runtime.update()

    print(
        "[project] simulation is playing; "
        "press Ctrl+C to stop"
    )

    frame_count = 0
    while simulation_app.is_running() and not stop_requested:
        if manipulation_executor is None:
            simulation_app.update()
        else:
            manipulation_executor.spin()
        scenario_runtime.update()
        tray_runtime.update()
        frame_count += 1
        if frame_count == 60:
            print(
                "[project] timeline "
                f"playing={timeline.is_playing()} "
                f"time={timeline.get_current_time():.3f}",
                flush=True,
            )
    if timeline is not None:
        timeline.stop()
        simulation_app.update()

except BaseException as error:
    if stop_requested:
        print(
            f"[project] shutdown completed after {type(error).__name__}",
            flush=True,
        )
    else:
        print(
            "[project] fatal runtime error: "
            f"{type(error).__name__}: {error!r}",
            flush=True,
        )
        raise

finally:
    if tray_runtime is not None:
        tray_runtime.close()
    if scenario_runtime is not None:
        scenario_runtime.close()
    if scenario_node is not None:
        ros_bridge.shutdown(scenario_node)

    simulation_app.close()
    COMPOSED_STAGE.unlink(missing_ok=True)
