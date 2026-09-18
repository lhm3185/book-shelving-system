# `feature/1st_combine_integration_test` 검토 — FSM 시스템 구성에 로봇팔 맞추기

| 항목 | 값 |
| --- | --- |
| 작성 | 2026-09-18, D 김도윤 |
| 대상 | `origin/feature/1st_combine_integration_test` (f88d2a4, 이현민이 system-fsm + vision + 우리 `robot_control`(e667673 까지) 을 합침) |
| 결론 | **로봇팔 인터페이스는 FSM 구성과 맞는다.** 다만 **비전의 `/detect_target_slot` 액션 서버가 없어서** 지금 상태로는 FSM → 로봇팔까지 못 간다. 그리고 mock 값으로는 우리 노드가 M410 으로 거절한다 |

## 1. FSM 시스템 구성 (읽은 대로)

```
무인반납기 → /return_machine/tray_job (TrayJob)
    ↓
task_manager_node (상태기계)
  INITIALIZING → IDLE → PLANNING → NAV_TO_RETURN → RECEIVE_TRAY
    → SELECT_BOOK → NAV_TO_SHELF → DETECT_TARGET_SLOT → PLACE_BOOK
    → UPDATE_DATA → NEXT_BOOK(반복) → RETURN_HOME → COMPLETED
  실패하면 FAILED / WAIT_FOR_OPERATOR
    ↓ 액션 3개
  /navigate_to_target (AMR)   /detect_target_slot (비전)   /place_book (로봇팔 = 우리)
```

- 상태·오류는 `/robot/status` (`RobotStatus`) 로 발행. `component` 로 구분 (FSM 은 `task_manager`, 우리는 `manipulation`)
- 설정은 `shelving_system/config/` 의 `system.yaml`(토픽·액션·프레임·시간 제한), `shelf_map.yaml`(서가 위치·분류), `book_profiles.yaml`(책 치수)
- **launch 파일 3개(`all`, `pc_a`, `pc_b`)는 아직 비어 있다** (0 바이트)

## 2. 우리(로봇팔)가 맞춰야 하는 계약 — 대조표

| FSM 쪽 | 값 | 우리 노드 | 판정 |
| --- | --- | --- | --- |
| 액션 이름 | `/place_book` | 같음 | ✅ |
| 액션 정의 | `PlaceBook.action` (goal 7개, result 5개, feedback 2개) | 같은 인터페이스 사용 | ✅ |
| 상태 토픽 | `/robot/status` `RobotStatus` | 같은 토픽에 `component: manipulation` 발행 | ✅ |
| 좌표 프레임 | `frames.arm_base = arm_base_link` | goal 은 `arm_base_link` 만 받음 | ✅ |
| 작업 시간 제한 | `timeouts.manipulation = 60.0` s | 1작업 21.75 s(시뮬), 벽시계 8 s | ✅ 여유 |
| 재시도 | `retry_limits.manipulation = 1` | 재시도 가능 오류 구분해서 보고(405·407·409 등) | ✅ |
| goal 채우는 값 | `job_id`, `book_id`, `target_slot`(비전 결과 그대로), `book_width/height/thickness`(프로파일), `insertion_speed`(파라미터 0.03) | 전부 사용. 치수 0 이면 기본 프로파일 | ✅ |
| 삽입 속도 | 0.03 m/s | `max_insertion_speed 0.05` 이내 | ✅ |
| 결과 판정 | `success and placement_verified` 여야 성공, 아니면 `error_code` 를 4000번대로 감쌈 | 우리는 0 / 401~412 를 낸다 | ✅ |
| 피드백 | `phase`, `progress` 로그만 함 | 9단계 이름으로 발행 | ✅ |

**결론: 우리 쪽은 인터페이스 변경 없이 그대로 붙는다.**

## 3. 지금 상태로 통합하면 막히는 곳 (2건)

### (1) 비전에 `/detect_target_slot` 액션 서버가 없다 — **1순위**

- `shelving_perception` 의 실행 파일은 `vision_manager` 하나뿐이고, `DetectTargetSlot` 을 쓰는 코드가 없다
- `perception.yaml` 에서 **서가 목표 관련 설정이 전부 주석 처리**돼 있다 ("Shelf targeting is disabled for the book-only validation stage")
- FSM 은 `DETECT_TARGET_SLOT` 상태에서 이 액션을 호출하므로 → `ERROR_PERCEPTION_SERVER(3001)` 로 끝난다
- 지금 동작하는 것은 `shelving_system/mock_perception_server.py` 뿐

### (2) mock 이 주는 칸 값은 우리 노드가 거절한다 (9/17 확인, 그대로 남아 있음)

| mock 값 | 우리 판정 |
| --- | --- |
| 위치 (0.55, 0, 0.80) | **M410** 검증 범위 밖 (x −0.56~−0.22, y 0.45~0.65, z 0.25~0.45) |
| 자세 단위 쿼터니언(w=1) | **M410** 삽입 방향 yaw 0° (1차 지원 90°±6°) |
| 1차 칸 (−0.3497, 0.5495, 0.3399), z=w=0.7071068 로 바꾸면 | **통과** |

→ 세션에서 **mock 두 값만 고치면** FSM→로봇팔 관통이 바로 된다.

## 4. 그밖에 맞춰 둘 것 (우리 쪽 조정 후보)

| 항목 | FSM 값 | 우리 값 | 의견 |
| --- | --- | --- | --- |
| 책 프로파일 | `standard` 0.18 × 0.24 × 0.035 | 시뮬 실제 책 0.163 × 0.237 × 0.035 | 폭이 1.7 cm 큼. Isaac 은 실제 AABB 로 잡으므로 동작엔 영향 없지만, **칸 폭 검사를 켜면**(`require_slot_dimensions`) 어긋난다. 지금은 꺼 둠 |
| 칸 신뢰도 | `minimum_slot_confidence 0.70` (FSM 이 먼저 거른다) | `min_confidence 0.0` | 그대로 두어도 됨 — FSM 이 앞에서 거름 |
| 한 트레이 최대 | `maximum_books_per_tray 5` | 트레이 6칸 | 6칸 중 5권까지만 쓰는 것으로 이해. 1차 시연 4권이라 문제 없음 |
| 카메라 프레임 | `system.yaml frames.camera = camera_link` | 실제 발행은 `sim_camera` / `Camera_OmniVision_OV9782_Color` | **이름이 다르다.** FSM 이 이 값을 쓰기 시작하면 맞춰야 함 (현재는 사용처 없음) |
| launch | `pc_a/pc_b/all` 비어 있음 | 우리 실행 스크립트로 대신 | 이현민이 채울 때 **우리 노드 실행 조건**(`executor:=sim`, 정적 TF `panda_link0→arm_base_link`)을 전달해야 함 |
| 우리 최신 커밋 | e667673 까지만 합쳐짐 | 이후 6845d70 ~ bc158f0 (문서·여러 종류 책·학습 도구) | 다음 통합 때 다시 합쳐야 함 |

## 5. 세션에서 할 순서 (제안)

1. mock 칸 값 2개 수정 → FSM → 로봇팔 1권 관통 확인 (`FSM_PLACEBOOK_SESSION.md` 체크리스트)
2. 실패 경로 확인: Isaac 끈 상태(M411), 잘못된 goal(M410), 취소(M412) 를 FSM 이 4000번대로 감싸는지
3. 사이클 시간 측정 (기준 21.75 s 시뮬)
4. 비전 `/detect_target_slot` 서버 일정 확인 — 1차 시연에 넣을지, mock 으로 갈지 결정
5. launch 파일에 우리 노드 실행 조건 반영 요청
