# ROS2 인터페이스 명세

## 1. 목적

이 문서는 다섯 ROS2 패키지가 서로 주고받는 최소 인터페이스를 정의한다. 각 담당자는 다른 패키지의 내부 Python 파일을 직접 불러오지 않고 이 인터페이스만 사용한다.

## 2. 공통 규칙

- 모든 작업에 중복되지 않는 job_id를 사용한다.
- 책 한 권마다 book_id를 사용한다.
- 좌표는 frame_id와 timestamp를 포함한다.
- 길이는 미터, 각도는 라디안을 사용한다.
- 성공 여부와 함께 error_code와 사람이 읽을 수 있는 message를 제공한다.
- 시간이 오래 걸리는 이동·인식·삽입은 ROS2 action으로 구현한다.
- 액션 취소 시 즉시 모터 명령만 끊지 않고 정의된 안전 상태로 이동한 뒤 결과를 반환한다.

## 3. TrayJob.msg

무인반납기에서 공급된 책 묶음을 표현한다.

| 필드 개념 | 설명 |
| --- | --- |
| job_id | 전체 작업 ID |
| tray_id | 트레이 ID |
| book_ids | 책 ID 목록 |
| rfid_tags | 가상 RFID 목록 |
| classification_codes | 책별 분류코드 |
| created_at | 작업 생성 시각 |

1차에서는 return_machine_node가 이 메시지를 발행하고 task_manager_node가 구독한다.

## 4. RobotStatus.msg

전체 시스템이 PC B의 현재 상태를 확인하기 위한 메시지다.

| 필드 개념 | 설명 |
| --- | --- |
| component | perception, navigation 또는 manipulation |
| state | IDLE, RUNNING, SUCCEEDED, FAILED |
| active_job_id | 현재 작업 |
| progress | 0.0~1.0 진행률 |
| error_code | 오류 코드 |
| message | 현재 상태 설명 |
| heartbeat_time | 마지막 상태 갱신 |

## 5. TargetSlot.msg

비전이 발견한 삽입 가능한 빈 공간의 결과다.

| 필드 개념 | 설명 |
| --- | --- |
| frame_id | 좌표 기준, 최종 전달은 arm_base_link 권장 |
| position | 빈 공간 중심 |
| orientation | 책 삽입 방향 |
| available_width | 사용 가능한 폭 |
| available_height | 사용 가능한 높이 |
| insertion_depth | 권장 삽입 깊이 |
| pre_insert_offset | 사전 삽입 거리 |
| confidence | 0.0~1.0 신뢰도 |
| detection_time | 촬영·계산 시각 |

## 6. NavigateToTarget.action

AMR 이동과 작업 위치 정렬에 사용한다.

### Goal

- job_id
- target_type: RETURN_STATION, SHELF, HOME
- target_id
- map 기준 목표 pose
- position_tolerance
- yaw_tolerance

### Feedback

- 현재 이동 단계
- 남은 거리
- position error
- yaw error
- 재시도 횟수

### Result

- 성공 여부
- 최종 base pose
- 허용오차 충족 여부
- error_code와 message

## 7. DetectTargetSlot.action

서가의 빈 공간을 탐지하고 로봇팔이 사용할 pose를 생성한다.

### Goal

- job_id와 book_id
- shelf_id
- 대상 책 폭·높이·두께
- 최소 안전 여유
- 최대 재촬영 횟수

### Feedback

- CAPTURING
- DETECTING_SHELF
- FINDING_EMPTY_SPACE
- CALCULATING_POSE
- TRANSFORMING_FRAME
- 현재 후보 수와 최고 신뢰도

### Result

- 성공 여부
- TargetSlot
- 검토한 후보 수
- 실패 원인
- error_code와 message

## 8. PlaceBook.action

트레이 책 인식, 파지와 빈 공간 삽입을 요청한다.

### Goal

- job_id와 book_id
- TargetSlot
- 책 크기 또는 book profile ID
- 삽입 속도 제한

### Feedback

- DETECTING_BOOK
- PLANNING_GRASP
- APPROACHING_BOOK
- GRASPING
- MOVING_TO_PRE_INSERT
- INSERTING
- RELEASING
- RETREATING
- VERIFYING

### Result

- 성공 여부
- 실패 단계
- 최종 배치 확인 결과
- error_code와 message

## 9. 오류 코드 범위

| 범위 | 기능 | 예 |
| --- | --- | --- |
| C1xx | System/FSM | 잘못된 상태 전이, 중복 작업 |
| P2xx | Perception | 서가 미검출, 빈 공간 없음, 낮은 신뢰도 |
| N3xx | Navigation | 경로 실패, 정렬 오차 초과 |
| M4xx | Manipulation | IK 실패, 파지 실패, 삽입 충돌 |
| Y5xx | YAML/Data | 서가 정보 없음, 상태 저장 실패 |
| S6xx | Simulation | 트레이 생성 또는 시나리오 초기화 실패 |

## 10. 패키지 의존 방향

~~~text
shelving_interfaces
   ↑        ↑        ↑        ↑
system  perception  navigation  manipulation
~~~

네 실행 패키지는 interfaces에만 의존한다. perception이 manipulation 내부 파일을 직접 호출하거나 system이 navigation Python 모듈을 import하지 않는다.

## 11. 인터페이스 변경 절차

1. 변경이 필요한 이유와 사용 예시를 기록한다.
2. 영향을 받는 담당자와 필드·단위·좌표계를 합의한다.
3. shelving_interfaces를 먼저 수정한다.
4. 각 담당자가 자신의 action client 또는 server를 갱신한다.
5. PC A와 PC B 통신 시험 후 통합 브랜치에 반영한다.

