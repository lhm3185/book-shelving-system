# 좌표 계약 (전원 준수, 변경은 4인 합의)

| 항목 | 값 |
| --- | --- |
| 작성 | 2026-09-17, D 로봇팔 (초안 — 비전 담당과 확정) |
| 근거 | 웹 클로드 v7 회신 B절 "같은 유형이 세 번 반복되면 개별 실수가 아니라 계약 부재" |
| 적용 대상 | `TargetSlot`, `PlaceBook`, `DetectTargetSlot`, `/perception/*` 좌표, 트레이 칸 설정값 |
| 로봇팔 세부 | `FRAMES_CONTRACT.md` (Franka 프레임 매핑, stow) |

---

## 1. 다섯 줄

```
1. 기준 프레임  arm_base_link                      (base_link, sim_camera, world 아님)
2. 물체 기준점  AABB(바운딩박스) 중심               (에셋 원점 아님, 윗면·표면 한 점 아님)
3. 축 규약      REP-103  +X 앞, +Y 왼쪽, +Z 위
4. 단위         m, rad
5. 변환 책임    발행하는 쪽이 변환해서 낸다. 변환은 코드가 아니라 TF 로 한다
```

**5번이 핵심이다.** 받는 쪽은 frame_id 가 `arm_base_link` 가 아니면 쓰지 않는다 (로봇팔은 M410 으로 거절).

---

## 2. 왜 필요한가 — 실제로 난 사고

| # | 시점 | 사고 | 크기 | 유형 |
| --- | --- | --- | --- | --- |
| 1 | 9/16 | 책 에셋 `C_book002_low` 원점이 책 밖 | 92 mm | 원점 ≠ 물체 중심 |
| 2 | 9/17 | 책 USD 원점이 실제 책에서 떨어짐 → 허공 파지 | **37 cm** | 원점 ≠ 물체 중심 |
| 3 | 9/17 | 비전 책 좌표 = 박스 중심 픽셀 깊이(윗면) ↔ 로봇팔 = AABB 중심 | 8 cm | 기준점 불일치 |
| 4 | 9/17 | 손목 카메라 광학 프레임에 180° 를 한 번 더 붙임 → 책이 카메라 **위**로 계산됨 | **0.8 m** | 축 규약 이중 적용 |
| 5 | 9/17 | 비전 박스 중심 픽셀이 책 사이 틈에 떨어져 바닥 깊이를 읽음 | 최대 0.67 m | 표면 한 점 사용 |

1·2 는 에셋만 고치면 됐다. **3·4·5 는 양쪽 코드가 각자 규약으로 맞게 동작하면서 서로 다른 것을 가리켰다.**
단위 시험은 양쪽 다 통과한다. 통합해야 드러난다.

---

## 3. 프레임 트리 (1차, Isaac Sim)

```
panda_link0 ──(정적, 항등)──▶ arm_base_link              ← 모든 좌표의 기준
panda_link0 ──(Isaac TF)────▶ wrist_camera              ← Isaac 이 발행
wrist_camera ─(정적, 항등)──▶ wrist_camera_optical_frame ← 이미지 frame_id
panda_hand ───(정적, 항등)──▶ tool0
panda_hand ───(정적, z+0.10, z축 180°)──▶ gripper_tcp
```

| 프레임 | 축 | 누가 발행 | 확인 방법 (2026-09-17) |
| --- | --- | --- | --- |
| `arm_base_link` | REP-103 | 로봇팔 정적 TF (`arm_frames.launch.py`) | Isaac 에서 `panda_link0` 월드 회전 = 항등 (쿼터니언 1,0,0,0) |
| `wrist_camera` | **광학 규약 (+Z 광축, +X 오른쪽, +Y 아래)** | Isaac `ROS2PublishTransformTree` | TF 값에서 +z 가 트레이 쪽(아래)을 향함 |
| `wrist_camera_optical_frame` | 광학 규약 | 정적 TF, **항등** | 책 윗면 높이 오차 0.9 mm |

### 반드시 알아둘 Isaac 동작 (설치본 5.1.0 실측)

| 항목 | 사실 | 틀리면 |
| --- | --- | --- |
| 카메라 TF 축 | `ROS2PublishTransformTree` 는 카메라 prim 을 **이미 광학 규약으로** 발행한다. USD 카메라 축(-Z 앞)이 아니다 | 180° 를 더 붙이면 z 가 뒤집혀 0.8 m 틀어진다 (사고 4) |
| 이미지 frame_id 기본값 | `ROS2CameraHelper.frameId` = `sim_camera` | TF 트리에 없는 이름 → 변환 실패 |
| 로봇 전체 TF | 로봇 루트를 대상으로 넣으면 ridgeback_franka 트리가 **고리**가 된다 (`world→panda_link2→…→base_link→…→world`) | tf2 "tree contains a loop", 모든 변환 실패 |
| 시각 | 센서·TF 모두 sim time (`useSystemTime` 기본 False) | 노드가 `use_sim_time:=true` 가 아니면 시각 조회 실패 |

---

### 시뮬 상태를 읽는 시점 (같은 유형 3회 반복 — 웹 클로드 v9 회신 B2)

```
시뮬 상태를 설정한 직후에 읽지 말 것.
자세를 바꿨으면 최소 1 물리 스텝(렌더 포함)을 돌린 뒤 읽거나, 설정한 값을 순기구학으로 직접 계산해서 쓸 것.
그래프 설정(발행 주기, TF 대상)은 재생 전에 USD 로 넣을 것 — 실행 중 OmniGraph 로 바꾸면 반영되지 않는다.
```

| 시점 | 증상 | 근본 |
| --- | --- | --- |
| v6 | 실행 중 매 스텝 IK 를 새로 풀어 해 가지 전환 → 관절 튐 | **언제 계산하느냐** |
| v7 | `MoveJoint` 가 "실제 위치 + 한 걸음" 지령 → 잔여 오차 안 줄어듦 | **무엇을 읽어 지령을 만드느냐** |
| v9 | 순간이동 직후 USD 자세가 옛 값(손 z 3.38 m) → 관측 자세 후보 전부 "화면 밖" | **언제 읽느냐** |
| v9 | 실행 중 바꾼 카메라 주기·TF 대상이 무시됨 | **언제 설정하느냐** |

---

## 4. 메시지별 규약

### 4-1. `TargetSlot` (비전 → FSM → 로봇팔)

| 필드 | 의미 | 로봇팔 검사 |
| --- | --- | --- |
| `header.frame_id` | `arm_base_link` | 다르면 **M410** |
| `header.stamp` | 촬영 시각 (sim time) | 1차는 검사 끔 (`max_target_age_s: 0`) |
| `pose.position` | **꽂힌 뒤 책의 AABB 중심** | 검증 범위 밖이면 M410 |
| `pose.orientation` | **+X 축 = 삽입 방향** (서가 안쪽). 1차 서가는 arm_base_link +Y → yaw +90° (z=0.7071, w=0.7071) | yaw 90°±6°, 기울기 ±6° 밖이면 M410. **단위 쿼터니언(w=1)은 yaw 0° 라 거절된다** |
| `available_width` | 칸의 빈 폭 | 주면 `max(책 두께, 닫힌 손가락 0.0528) + 0.01` 이상. 0 이면 1차는 미제공으로 봄 |
| `available_height` | 칸의 빈 높이 | 주면 책 높이 + 0.02 이상 |
| `confidence` | 0~1 | 1차 기준 0 (고정 칸) |

1차 고정 칸 4곳 (Isaac 레벨 v3, 트레이 6칸 × 4곳 전부 계획 통과):
`x ∈ {-0.5097, -0.4297, -0.3497, -0.2697}`, `y = 0.5495`, `z = 0.3399`

### 4-2. `PlaceBook` 책 치수

| 필드 | 뜻 | 예 (Isaac 책) |
| --- | --- | --- |
| `book_thickness` | 책등 폭 (두께) | 0.035 |
| `book_height` | 책등 길이 (세웠을 때 높이) | 0.237 |
| `book_width` | 책등 → 앞마구리 (서가에서 깊이) | 0.163 |

셋 다 0 이면 `book_profiles.yaml` 기본 프로파일. 일부만 0 이면 M410.

### 4-3. 트레이 책 좌표 (1차: 설정값, 비전은 그림자 모드)

- `shelving_manipulation/config/book_profiles.yaml` 의 `tray.slots[].center` = **트레이에 선 책의 AABB 중심**, arm_base_link
- 비전이 같은 책을 검출하면 **같은 기준점(AABB 중심)** 으로 변환해서 기록한다. 책등 윗면 점이면 `z − 책폭/2`

### 4-4. `/perception/books` (비전, 현재)

| 현재 | 계약 |
| --- | --- |
| `PointStamped`, 박스 중심 픽셀 깊이 = **책 표면 한 점** | 책 AABB 중심. 마스크 안 깊이 중앙값 + 책 치수로 보정 |
| frame_id 파라미터 (`sim_camera` 등) | 최종 전달은 `arm_base_link` |

---

## 5. 확인 절차 — 통합 전에 한 번씩

| 확인 | 방법 | 합격 |
| --- | --- | --- |
| 프레임 이름 | `ros2 run tf2_ros tf2_echo arm_base_link wrist_camera_optical_frame` | 값이 나온다 (고리·끊김 없음) |
| 카메라 축 | 카메라가 아래를 볼 때 위 TF 의 회전에서 광학 +z 가 arm_base_link −z 쪽 | 방향 일치 |
| 알려진 물체 | 트레이 칸 책을 비전으로 검출 → arm_base_link 변환 → `book_profiles.yaml` 값과 비교 (`isaac_sim/tools/books_eval.py`) | 오차 수 cm 이내 (z 는 기준점 보정 후) |
| 시각 | 모든 노드 `use_sim_time:=true`, `/clock` 수신 | TF 조회 extrapolation 경고 없음 |

---

## 6. 결정이 필요한 것

| # | 항목 | 제안 |
| --- | --- | --- |
| 1 | 카메라 장착 위치 (레벨에 아직 없음) | 손목. 장착 pose 는 `panda_hand → wrist_camera` 정적 값으로 고정해 문서화 |
| 2 | 02 문서 TF 트리 | `base_link → camera_link` 를 `tool0 → wrist_camera → wrist_camera_optical_frame` 로 수정 (v7 회신 A3) |
| 3 | 비전 좌표 발행 frame | `sim_camera` → `arm_base_link` (TF 대기 문제 수정 후, `VISION_ISAAC_TEST.md` 3-3) |
