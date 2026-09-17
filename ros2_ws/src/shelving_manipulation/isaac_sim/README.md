# isaac_sim — Isaac Sim 쪽 로봇팔 실행 코드

ROS 패키지(`shelving_manipulation`)와 **같은 담당(D)의 코드**지만 ROS 노드가 아니다.
Isaac Sim 내장 Python(3.11)에서 돌고, 설치(`colcon build`) 대상이 아니다. lint 대상에서도 제외했다 (`test/test_flake8.py`, `test/test_pep257.py`).

| 경로 | 내용 |
| --- | --- |
| `arm_primitives.py`, `arm_config.yaml` | 동작 단위(MoveJoint·MoveLinear·SetGripper·Wait·Sequence)와 ArmController. 스텝 구동, 블로킹 없음 |
| `arm_planning.py`, `arm_geometry.py`, `book_tasks.py` | 사전 경로 계획(5mm·2° 이산 IK), 접어 돌기(`tucked_joint_moves`), 쿼터니언 |
| `arm_errors.py`, `arm_mock.py` | M4xx, Isaac 없이 시험하는 mock 백엔드 |
| `tests/` | 위 순수 모듈 단위시험 (`python3 -m pytest isaac_sim/tests`) |
| `isaac/place_book_server.py` | **PlaceBook 작업 실행기.** `manipulation_node`(executor:=sim)의 JSON 명령을 받아 실행 |
| `isaac/book_scene.py` | 레벨 + 트레이 + 책 + 북엔드 장면, 계획, 실행 조립 |
| `isaac/ros_sensors.py` | 손목 카메라(가정) + `/rgb /depth /camera_info /tf /clock` 발행 그래프 |
| `isaac/run_place_book_server.sh` | 위 실행기를 ROS 환경변수와 함께 실행 (`--camera`, `--gui`) |
| `isaac/pick_place_book.py`, `multi_book.py` | 1권·여러 권 검증 스크립트 (기준선 측정) |
| `isaac/probe_*.py`, `verify_stow.py` 등 | 레벨·프레임·도달 범위·관측 자세 탐색 |
| `isaac/make_tray.py`, `tray_fit.py` | Blender 파라메트릭 트레이 제작·책 맞춤 검사 |
| `tools/` | 비전 연동 시험(`run_vision_test.sh`, `books_eval.py`, `grab.py`), 가짜 카메라 발행기 |

## GPU PC 배치

스크립트는 GPU PC 의 `~/arm/` 에 있다고 가정한다 (예: `~/arm/isaac/place_book_server.py`, `~/arm/arm_config.yaml`).
`book_placer.py` 는 `~/shelving_manipulation_py/shelving_manipulation/` 에서 import 한다.

```bash
rsync -a --exclude=__pycache__ isaac_sim/ rokey@<GPU PC>:~/arm/
rsync -a --exclude=__pycache__ shelving_manipulation/ rokey@<GPU PC>:~/shelving_manipulation_py/shelving_manipulation/
```

실행 방법·결과: `docs/doyoon-kim/PLACEBOOK_SIM_LINK.md`, `VISION_ISAAC_TEST.md`
