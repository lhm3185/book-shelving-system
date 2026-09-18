# 비전 빈 칸 판정 — 새 방식(slot_positions) 시험 결과 (2026-09-18 밤, 비전 담당 전달용)

`origin/vision` `477d0f6` 를 Isaac 에 붙여 돌렸습니다. **판정 로직까지 가지 못하고 TF 단계에서 멈춥니다.**

## 1. 증상

```
Could not transform sim_camera to arm_base_link: Lookup would require extrapolation into the past.
Requested time 262.450014 but the earliest data is at time 270.000014
```

같은 `Requested time 262.450014` 가 계속 반복됩니다.

## 2. 원인 — TF 문제가 아니라 **오래된 프레임을 계속 붙잡고 있는 것**

| 값 | 시각(시뮬) |
| --- | --- |
| 노드가 변환을 요청한 프레임 | **262.45** |
| TF 버퍼에 남은 가장 오래된 데이터 | 270.00 |
| 그때 현재 시각 | 510 |

- **TF 자체는 정상입니다.** `tf2_echo arm_base_link sim_camera` → `[-0.476, 0.119, 0.588]`
- **토픽 시각도 서로 맞습니다.** `/rgb` `/depth` `/camera_info` 471.550, `/tf` 471.600, 당시 sim 471
- 즉 노드가 들고 있는 프레임이 **TF 버퍼(기본 10초)보다 오래돼서** 변환이 영원히 실패하고, 실패해도 **같은 프레임으로 계속 재시도**해 회복되지 않습니다

### 서가 모델을 받아 다시 시험해도 같았다 (2026-09-18 밤, 2차)

`best.pt`(서가 segmentation, 클래스 `shelf_closed`/`shelf_filled`/`shelf_open`)를 받아 그대로 넣고 다시 돌렸습니다.

| 확인 | 결과 |
| --- | --- |
| 모델 로드 | 정상 (segment, 클래스 3개) |
| 증상 | **동일** — 같은 TF 경고 반복 |
| 경고 7개의 요청 시각 | **전부 `83.350004` 로 같음** |

**요청 시각이 하나로 고정돼 있다는 것이 결정적입니다.** 노드가 특정 프레임 하나를 붙잡고 계속 재시도하고 있으며,
그 프레임은 TF 버퍼에 남아 있지 않을 만큼 오래된 것입니다. 서가 모델 유무와는 무관합니다.

## 3. 제안

1. 트리거를 받으면 **그 시점의 최신 프레임**으로 처리하고, 오래된 프레임(예: 1초 이상)은 버립니다
2. 변환 실패 시 경고 한 번만 남기고 **다음 프레임으로 넘어갑니다** (같은 프레임 재시도 금지)
3. 필요하면 `buffer.lookup_transform(..., timeout=Duration(seconds=0.2))` 로 잠깐 기다립니다

## 4. 확인한 것 / 못 한 것

| | 내용 |
| --- | --- |
| 확인됨 | 노드 기동, 파라미터 로드, `/detect_target_slot` 액션 서버 등록, TF·토픽 정합 |
| 못 함 | 빈 칸 판정 결과 — `/perception/empty_shelf_position` 미발행 (위 문제로 판정 단계까지 못 감) |
| 대체 | 서가 모델(`260918_train/best.pt`)이 저희 쪽에 없어 **책 모델로 대체**해 실행했습니다. 서가 검출 정확도는 이번 시험 대상이 아니었습니다 |

## 5. 재현 방법

```bash
# GPU PC — Isaac (카메라 포함)
cd ~/book-shelving-system && ./scripts/run_isaac_sim.sh --headless --book-variants mixed \
  --camera-prim /World/ridgeback_franka/panda_hand/rsd455/RSD455/Camera_OmniVision_OV9782_Color

# 이 PC — 정적 TF 2개 + 비전 노드
ros2 run tf2_ros static_transform_publisher --frame-id panda_link0 --child-frame-id arm_base_link --ros-args -p use_sim_time:=true
ros2 run tf2_ros static_transform_publisher --frame-id Camera_OmniVision_OV9782_Color --child-frame-id sim_camera --ros-args -p use_sim_time:=true
ros2 run shelving_perception vision_manager --ros-args --params-file <perception.yaml> \
  -p model_path:=<책 모델> -p shelf_model_path:=<서가 모델> -p show_debug_window:=false
ros2 topic pub --once /perception/detect_request std_msgs/msg/Bool "{data: true}"
```

## 6. 좌표 규약은 맞습니다

`slot_positions` 4개 좌표와 orientation(z=w=0.7071068) 모두 로봇팔이 검증한 값과 같습니다. 그대로 두시면 됩니다.

## 7. 지난 지적사항 반영 확인

9/18 오후에 전달한 3가지가 새 방식에서 모두 해소됐습니다 (`SHELF_GAP_CHECK_20260918.md`).

| 지적 | 새 방식 |
| --- | --- |
| 한 단 전체 중앙값이 섞인다 | 후보별 개별 검사로 변경 |
| 5등분이 선반판과 어긋난다 | 5등분 폐지, bbox 는 표시용 |
| 기준 깊이가 로봇 팔 때문에 오염된다 | 후보 주변 비율 + 상·하한 파라미터 |
