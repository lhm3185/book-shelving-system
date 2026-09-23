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
        "nav_map_yaml": scene.get("nav_map_yaml"),
        "robot.usd": scene.get("robot", {}).get("usd"),
        "robot.prim_path": scene.get("robot", {}).get("prim_path"),
    }
    missing = [name for name, value in required.items() if not value]
    if missing:
        raise ValueError(f"Missing scene fields: {', '.join(missing)}")

    for label in ("world_usd", "nav_map_yaml"):
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
ROBOT_USD = resolve_project_path(SCENE["robot"]["usd"])
ROBOT_PRIM_PATH = SCENE["robot"]["prim_path"]
ROS_DOMAIN_ID = int(SCENE.get("ros", {}).get("domain_id", 0))


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
from isaacsim.core.utils import stage as stage_utils


stop_requested = False


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

    for graph_path in ("/Graphs/Nav2BaseController", "/Graphs/LidarSensorGraph"):
        if not stage.GetPrimAtPath(graph_path).IsValid():
            raise RuntimeError(f"Required graph is missing: {graph_path}")
        graph = og.get_graph_by_path(graph_path)
        if not graph.is_valid():
            raise RuntimeError(f"OmniGraph runtime object is invalid: {graph_path}")
        was_disabled = graph.is_disabled()
        if was_disabled:
            graph.set_disabled(False)
        print(
            f"[project] graph={graph_path} "
            f"was_disabled={was_disabled} disabled={graph.is_disabled()}",
            flush=True,
        )

    physics_scene_path = "/World/PhysicsScene"
    if not stage.GetPrimAtPath(physics_scene_path).IsValid():
        raise RuntimeError(f"Required PhysicsScene is missing: {physics_scene_path}")
    print(f"[project] physics_scene={physics_scene_path}")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()
    print("[project] simulation is playing; press Ctrl+C to stop")

    frame_count = 0
    while simulation_app.is_running() and not stop_requested:
        simulation_app.update()
        frame_count += 1
        if frame_count == 60:
            print(
                "[project] timeline "
                f"playing={timeline.is_playing()} "
                f"time={timeline.get_current_time():.3f}",
                flush=True,
            )

    timeline.stop()
    simulation_app.update()
finally:
    simulation_app.close()
    COMPOSED_STAGE.unlink(missing_ok=True)
