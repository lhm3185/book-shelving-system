# (이동됨) Isaac 실행 코드는 `isaac_sim/` 으로 옮겼다

2026-09-18 구조 변경. Isaac API 를 직접 쓰는 코드는 저장소 최상위 `isaac_sim/` 에서 관리한다.

| 예전 | 지금 |
| --- | --- |
| `isaac_sim/isaac/place_book_server.py` | `isaac_sim/isaac/run_simulation.py` + `controllers/manipulation_executor.py` |
| `isaac_sim/isaac/book_scene.py` | `isaac_sim/isaac/controllers/book_scene.py` (+ `world_loader.py`) |
| `isaac_sim/isaac/ros_sensors.py` | `isaac_sim/isaac/sensors/camera_bridge.py` |
| `isaac_sim/arm_*.py`, `book_tasks.py` | `isaac_sim/isaac/controllers/` |
| `isaac_sim/arm_config.yaml` | `isaac_sim/isaac/config/arm.yaml` |
| `isaac_sim/tools/`, `isaac/probe_*.py` | `isaac_sim/isaac/tools/` |
| `isaac_sim/tests/` | `isaac_sim/isaac/tests/` |
| `run_place_book_server.sh`, `run_demo_gpu.sh` | `scripts/run_isaac_sim.sh` (여기 두 파일은 호환용 껍데기) |

공식 실행:

```bash
./scripts/run_isaac_sim.sh --gui        # 또는 --headless
```

ROS2 Manipulation 기능 코드(`manipulation_node.py`, `grasp_planner.py`, `book_placer.py`)와 `config/`, `test/` 는 이 패키지에 그대로 남는다.
