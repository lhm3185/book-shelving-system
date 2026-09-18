# Isaac Sim 통합 실행 구조 변경 — 반영 결과 (이현민 전달용)

| 항목 | 값 |
| --- | --- |
| 작성 | 2026-09-18, D 김도윤 |
| 대상 | "Isaac Sim 통합 실행 구조 수정 요청" 전체 |
| 브랜치 | `feature/robot_control` (`792a8b1`). 검토 후 통합 브랜치로 병합 |
| 결과 | **요청한 구조로 옮기고, 새 실행기로 파지 2/2 성공 확인.** 다만 **통합 USD 가 비어 있어** 완료 기준 1개는 아직 못 채움 |

## 1. 한 일

```
simulation/
├── isaac/
│   ├── run_simulation.py          공식 진입점 (앱·USD·브리지·센서·실행기·루프)
│   ├── world_loader.py            USD 열기 + Prim 검사 (없으면 시작 시 오류)
│   ├── ros_bridge.py              브리지 활성화 · 노드 · 정리
│   ├── controllers/
│   │   ├── manipulation_executor.py   (place_book_server.py 의 로봇팔 부분)
│   │   ├── book_scene.py, arm_*.py, book_tasks.py
│   ├── sensors/camera_bridge.py   (ros_sensors.py)
│   ├── config/arm.yaml            (arm_config.yaml)
│   ├── tools/ (28개), tests/
└── scripts/run_isaac_sim.sh       공식 실행 명령 (--gui / --headless)
```

- `git mv` 로 옮겨 이력을 유지했습니다.
- 예전 실행 경로(`run_place_book_server.sh`, `run_demo_gpu.sh`)는 **호환용 껍데기**로 남겨 새 실행기를 부릅니다. `place_book_server.py` 는 삭제했습니다.
- 개인 절대경로 제거: 저장소 기준 경로(`__file__` 기준)로 계산하고, Isaac 설치 위치만 `ISAAC_SIM_PATH` 로 받습니다. USD 는 `--usd` 또는 `SIM_USD` 로 덮어쓸 수 있습니다.
- **`~/arm` 복사 없이** GPU PC 의 저장소에서 바로 실행되는 것을 확인했습니다 (`git pull` → `./scripts/run_isaac_sim.sh`).

## 2. 확인한 결과 (요청 9절 5단계 비교 시험)

GPU PC, 레벨 v5, 책 6종, 헤드리스, 카메라 prim 사용.

| 항목 | 기존 `place_book_server.py` | **새 `run_simulation.py`** |
| --- | --- | --- |
| 파지·삽입 | 2/2 성공 | **2/2 성공** (같은 칸 x 2.485 / 2.405) |
| 꽂힘 판정 | 5개 항목 통과 | **동일** |
| `/manipulation/sim/command` 수신 · `state` 발행 | 정상 | **동일** |
| `/rgb` | 10 Hz | **10.9 Hz** |
| `/clock`, `/depth`, `/camera_info`, `/tf` | 정상 | **정상** |
| `manipulation_node -p executor:=sim` | 연결 | **연결** |

**ROS 인터페이스는 하나도 바꾸지 않았습니다.** Task Manager 와 `PlaceBook.action` 수정 불필요합니다.

## 3. 아직 못 채운 완료 기준 — 통합 USD 가 비어 있음

```
simulation/library_system.usd      126 bytes (git-lfs 포인터, 실제 내용 1 byte)
simulation/assets/*.usd            모두 동일하게 빈 포인터
```

- 그래서 `simulation/library_system.usd` 를 기본으로 실행하면 **"USD 가 빈 파일이다" 라고 알리고 멈추도록** 했습니다 (조용히 실패하지 않게).
- 지금은 `SIM_USD=~/Desktop/ing_library_env_v5.usd` 로 시험용 레벨을 지정해 돌립니다.
- 트레이도 같은 이유로 `simulation/assets/tray.usd` 가 비어 있어, 없으면 예전 경로를 쓰도록 임시 처리해 두었습니다.

**결정이 필요합니다.** 통합 USD 를 채우는 방법은 둘 중 하나입니다.

| 방법 | 내용 | 비고 |
| --- | --- | --- |
| (가) 레벨 v5 를 저장소에 올린다 | 레벨 + 참조 에셋(책 112종·서가·바닥) 전부 LFS 로 | 용량이 큽니다(수백 MB). 제가 올릴 수 있습니다 |
| (나) 통합 USD 는 이현민이 새로 구성 | 서가·로봇·무인반납기를 통합 배치 | 로봇팔 파지 좌표를 **다시 검증**해야 합니다 (칸 좌표가 바뀌면 4/4 결과 무효) |

**시연이 9/21 이라, 1차까지는 v5 를 기준으로 가고 통합 USD 는 그 뒤에 채우는 쪽을 권합니다.**

## 4. 그밖에 확인된 것 (시연 전 정리 필요)

1. **`task_manager_node` 를 `--params-file system.yaml` 로 띄우면 죽습니다.** `system.yaml` 은 ROS 파라미터 파일 형식이 아니라 설정 파일입니다. 옵션 없이 띄워야 합니다
2. **`/place_book` 액션 서버가 2개면 시연이 깨집니다.** 오늘 실제로 겪었습니다 — 남아 있던 노드가 1 ms 만에 "트레이에 남은 책이 없음" 으로 응답하고 진짜 결과는 버려졌습니다. 시작 전에 `ros2 action info /place_book` → `Action servers: 1` 확인을 권합니다
3. **launch 3개(`all`, `pc_a`, `pc_b`)가 0바이트입니다.** 시연을 launch 로 띄울지, 스크립트로 띄울지 정해야 합니다
4. **mock 칸 좌표 순환** — 요청드린 대로 호출마다 4칸을 돌려주면 **4권 4/4 관통**이 됩니다 (오늘 로컬에서 검증). 코드는 아래 그대로입니다

```python
slots_x = [-0.3497, -0.4297, -0.5097, -0.2697]
self._slot_index = getattr(self, "_slot_index", 0)
target_slot.pose.position.x = slots_x[self._slot_index % len(slots_x)]
self._slot_index += 1
target_slot.pose.position.y = 0.5495
target_slot.pose.position.z = 0.3399
target_slot.pose.orientation.z = 0.7071068   # yaw +90°
target_slot.pose.orientation.w = 0.7071068
```

5. 책 프로파일이 FSM 0.18 × 0.24 vs 실제 0.163 × 0.237 로 다릅니다. 지금은 칸 폭 검사가 꺼져 있어 지나가지만 **켜면 M410** 입니다

## 5. 담당 경계 (요청 11절 그대로)

- `run_simulation.py`, `world_loader.py`, 통합 USD → **이현민**. 제가 초안을 만들어 두었으니 검토·인수해 주시면 됩니다
- `controllers/manipulation_executor.py`, `book_scene.py`, `arm_*` → 김도윤
- `controllers/navigation_executor.py` 자리 → 이동준 (인터페이스 경계만 비워 둠)
- `sensors/camera_bridge.py` → 윤재민 (현재 내용은 로봇팔 시험용 카메라·라이다 설정)
