# Isaac Sim 통합 실행 구조 변경 — 반영 결과 (이현민 전달용)

| 항목 | 값 |
| --- | --- |
| 작성 | 2026-09-18, D 김도윤 |
| 대상 | "Isaac Sim 통합 실행 구조 수정 요청" |
| 브랜치 | `feature/robot_control` (`72a9995`) — 검토 후 통합 브랜치로 병합 부탁드립니다 |
| 결과 | **요청하신 구조로 이동 완료. 새 실행기로 기존과 동일 동작(파지 2/2) 확인.** ROS 인터페이스는 변경 없음 |

전제: 지금 저장소의 통합 USD·에셋·mock 값은 **통신 시험용 자리**로 알고 작업했습니다. 아래 내용은 "빠졌다"가 아니라 **실제 내용으로 채울 때 필요한 사항**입니다.

## 1. 옮긴 구조

```
isaac_sim/isaac/
  run_simulation.py                    공식 진입점 (앱·USD·브리지·센서·실행기·루프)
  world_loader.py                      USD 열기 + Prim 검사
  ros_bridge.py                        브리지 활성화·노드·정리
  controllers/manipulation_executor.py 로봇팔 실행기 (place_book_server.py 의 팔 부분)
  controllers/navigation_executor.py   AMR 자리 (인터페이스 경계만)
  controllers/{book_scene, arm_geometry, arm_planning, arm_primitives, arm_errors, arm_mock, book_tasks}
  sensors/camera_bridge.py             (ros_sensors.py)
  config/arm.yaml                      (arm_config.yaml)
  tools/ (28개), tests/
scripts/run_isaac_sim.sh               공식 실행 (--gui / --headless)
scripts/run_isaac_tool.sh              같은 환경으로 tools 실행
```

- `git mv` 로 옮겨 파일 이력을 유지했습니다.
- 예전 실행 경로(`run_place_book_server.sh`, `run_demo_gpu.sh`)는 **호환용 껍데기**로 남겨 새 실행기를 부릅니다. `place_book_server.py` 는 삭제했습니다.
- 개인 절대경로를 기본값에서 제거했습니다. 저장소 기준(`__file__`) 경로를 쓰고, Isaac 설치 위치만 `ISAAC_SIM_PATH` 로 받습니다.
- **`~/arm` 복사 없이 저장소에서 직접 실행**되는 것을 GPU PC 에서 확인했습니다 (`git pull` → `./scripts/run_isaac_sim.sh`).

## 2. 확인 결과 (요청 9절 5단계 비교)

GPU PC, 헤드리스, 책 6종, 카메라 prim 사용.

| 항목 | 기존 `place_book_server.py` | 새 `run_simulation.py` |
| --- | --- | --- |
| 파지·삽입 | 2/2 성공 | **2/2 성공** (같은 칸 x 2.485 / 2.405) |
| 꽂힘 판정 5항목 | 통과 | **통과** |
| `/manipulation/sim/command` 수신 · `state` 발행 | 정상 | **동일** |
| `/rgb` | 10 Hz | **10.9 Hz** |
| `/clock` `/depth` `/camera_info` `/tf` | 정상 | **정상** |
| `manipulation_node -p executor:=sim` | 연결 | **연결** |

`/place_book`, `/manipulation/sim/*`, `/robot/status`, `PlaceBook.action` 모두 그대로입니다. Task Manager 는 수정할 것이 없습니다.

## 3. 실제 내용으로 채울 때 필요한 것

### 3-1. 통합 USD

현재 `isaac_sim/library_system.usd` 와 `isaac_sim/assets/*.usd` 는 자리 파일(1 byte)입니다. 실행기는 이 상태를 **조용히 지나치지 않고 "USD 가 빈 파일이다" 라고 알리고 멈추도록** 해 두었습니다. 지금은 아래처럼 시험 레벨을 지정해 돌립니다.

```bash
SIM_USD=~/Desktop/ing_library_env_v5.usd ./scripts/run_isaac_sim.sh --headless
```

통합 USD 를 채우실 때 로봇팔 쪽에서 필요한 것은 두 가지입니다.

| 항목 | 내용 |
| --- | --- |
| 필요한 Prim | `/World/ridgeback_franka`, `/World/bookshelves`, `/World/books` (없으면 시작 시 오류). 카메라·라이다·무인반납기는 있으면 쓰고 없으면 건너뜁니다 |
| **좌표 재검증** | 서가·로봇 배치가 v5 와 달라지면 **검증된 파지 좌표(서가 4칸, 트레이 6칸)가 무효**가 됩니다. 새 USD 를 주시면 재검증에 Isaac 30분이면 됩니다 |

제가 v5 레벨과 참조 에셋을 저장소에 올리는 방법도 가능합니다(수백 MB, LFS). 어느 쪽이 좋을지 정해 주시면 그대로 하겠습니다.

### 3-2. mock 인식 → 실제 인식으로 바꿀 때

지금 mock 이 고정 좌표를 주는 구조라 **여러 권을 넣으면 같은 칸에 꽂힙니다.** 시험용으로는 문제가 없지만, 시연에서 여러 권을 보여주려면 칸이 권마다 달라져야 합니다. 아래처럼 4칸을 순환하게 하면 **4권 4/4 관통**이 됩니다 (오늘 로컬에서 확인).

```python
slots_x = [-0.3497, -0.4297, -0.5097, -0.2697]   # 로봇팔이 검증한 1차 서가 칸
self._slot_index = getattr(self, "_slot_index", 0)
target_slot.pose.position.x = slots_x[self._slot_index % len(slots_x)]
self._slot_index += 1
target_slot.pose.position.y = 0.5495
target_slot.pose.position.z = 0.3399
target_slot.pose.orientation.z = 0.7071068   # yaw +90° (단위 쿼터니언은 로봇팔이 M410 으로 거절)
target_slot.pose.orientation.w = 0.7071068
```

실제 비전으로 바꾸실 때도 같은 규약입니다: `frame_id` 는 `arm_base_link`, 위치는 **꽂힌 뒤 책 AABB 중심**, 방향은 yaw +90°.

### 3-3. 책 프로파일

FSM 기본 프로파일은 0.18 × 0.24 × 0.035 이고 시뮬 실제 책은 0.163 × 0.237 × 0.035 입니다. 지금은 로봇팔의 칸 폭 검사가 꺼져 있어 그대로 통과하며, Isaac 은 실제 책 치수로 잡으므로 동작에는 영향이 없습니다. **검사를 켜실 계획이면** 값을 맞춰야 M410 이 나지 않습니다.

## 4. 실행할 때 알아두실 점 (오늘 겪은 것)

1. `task_manager_node` 는 `--params-file system.yaml` 로 띄우면 죽습니다. `system.yaml` 은 ROS 파라미터 파일 형식이 아니라 설정 파일이라 그렇습니다. **옵션 없이** 띄우면 정상입니다
2. `/place_book` 액션 서버가 **2개면 결과가 뒤섞입니다.** 남아 있던 노드가 먼저 응답하고 진짜 결과가 버려지는 일을 오늘 겪었습니다. 시작 전에 `ros2 action info /place_book` → `Action servers: 1` 확인을 권합니다
3. 같은 시험을 반복하면 `ros2 daemon` 이 죽은 노드 정보를 들고 있어 M411 이 계속 날 수 있습니다. `ros2 daemon stop` 후 재시도하면 풀립니다

## 5. 담당 경계 (요청 11절)

| 영역 | 담당 | 현재 상태 |
| --- | --- | --- |
| `run_simulation.py`, `world_loader.py`, 통합 USD | 이현민 | **제가 초안 작성 — 검토·인수 부탁드립니다** |
| `controllers/manipulation_executor.py`, `book_scene.py`, `arm_*` | 김도윤 | 유지 |
| `controllers/navigation_executor.py` | 이동준 | 자리만 비워 둠 |
| `sensors/camera_bridge.py` | 윤재민 | 현재는 로봇팔 시험용 카메라·라이다 설정 |
