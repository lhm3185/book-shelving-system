# 시스템 아키텍처와 정상 플로우

## 1. 전체 아키텍처

~~~mermaid
flowchart LR
    subgraph PCA[PC A]
      SIM[Isaac Sim]
      RETURN[무인반납기 가상 노드]
      FSM[Task Manager / FSM]
      PLAN[Job Planner]
      YAML[(YAML)]
      RETURN --> FSM
      FSM <--> PLAN
      PLAN <--> YAML
    end

    subgraph PCB[PC B]
      PER[Perception]
      NAV[Navigation / AMR]
      ARM[Manipulation / Robot Arm]
    end

    FSM -->|NavigateToTarget| NAV
    NAV -->|이동·정렬 결과| FSM
    FSM -->|DetectTargetSlot| PER
    PER -->|TargetSlot| FSM
    FSM -->|PlaceBook| ARM
    ARM -->|배치 결과| FSM
    SIM <--> NAV
    SIM <--> PER
    SIM <--> ARM
~~~

PC A는 작업 순서와 시뮬레이션을 관리하고, PC B는 로봇에 탑재된 컴퓨터로 간주한다. PC B의 세 기능은 자신의 알고리즘만 수행하고 전체 순서는 PC A의 FSM이 결정한다.

## 2. 정상 플로우

1. PC A에서 Isaac Sim과 shelving_system을 시작한다.
2. PC B에서 perception, navigation, manipulation 패키지를 시작한다.
3. Task Manager가 필수 노드와 상태를 확인하고 IDLE로 전이한다.
4. AMR은 주차 위치에서 작업을 대기한다.
5. 무인반납기 가상 노드가 책의 RFID와 분류코드를 생성한다.
6. 책을 파지 가능한 트레이에 담았다고 가정하고 TrayJob을 발행한다.
7. Task Manager가 중복 작업인지 확인한다.
8. Job Planner가 YAML에서 책의 분류코드에 맞는 목표 서가와 AMR 접근 waypoint를 조회한다.
9. AMR이 무인반납기 인수 위치로 이동한다.
10. AMR이 트레이 인수 위치에 정렬한다.
11. CP1에서 트레이 ID, 책 목록과 장착 상태를 확인한다.
12. 처리할 책 한 권을 선택한다.
13. AMR이 목표 서가의 관측 waypoint로 이동한다.
14. CP2에서 서가가 카메라 시야와 로봇팔 작업 범위에 들어오도록 AMR 자세를 정렬한다.
15. RGB-D 카메라로 서가를 촬영한다.
16. CP3에서 서가 영역, 선반 내부와 빈 공간 후보를 탐지한다.
17. 각 빈 공간의 폭·높이·깊이를 책 크기와 비교한다.
18. 삽입 가능한 공간 중 목표 정책에 맞는 하나를 선택한다.
19. 빈 공간 중심, 삽입 방향, 사전 삽입 pose와 권장 삽입 깊이를 계산한다.
20. camera_link의 결과를 arm_base_link 기준 좌표로 변환한다.
21. 트레이 위 책의 위치와 자세를 인식한다.
22. CP4에서 책 파지 pose와 삽입 경로를 생성한다.
23. 로봇팔이 트레이의 책을 파지한다.
24. 빈 공간 앞 사전 삽입 위치로 이동한다.
25. 책을 슬롯 방향에 정렬하여 저속으로 삽입한다.
26. 그리퍼를 해제하고 로봇팔을 안전 자세로 후퇴시킨다.
27. 카메라 또는 로봇 상태로 배치 완료를 확인한다.
28. 성공한 경우에만 YAML에 점유 상태와 결과를 기록한다.
29. 트레이에 책이 남아 있으면 다음 책을 선택하여 반복한다.
30. 모든 책을 처리하면 AMR이 주차 위치로 복귀하고 IDLE로 돌아간다.

## 3. Flow Chart

~~~mermaid
flowchart TD
    A[시스템 초기화] --> B{PC A/B 준비 완료?}
    B -- 아니오 --> B1[오류 기록 및 준비 재확인]
    B1 --> B
    B -- 예 --> C[AMR 주차 위치 대기]
    C --> D[TrayJob 수신]
    D --> E[YAML에서 목표 서가·접근 위치 조회]
    E --> F[무인반납기로 이동·정렬]
    F --> G[CP1 트레이 인수 확인]
    G --> H[다음 책 선택]
    H --> I[목표 서가 관측 위치로 이동]
    I --> J[CP2 카메라·팔 작업 위치 정렬]
    J --> K[RGB-D 서가 촬영]
    K --> L[CP3 서가·빈 공간 탐지]
    L --> M{책이 들어갈 공간인가?}
    M -- 아니오 --> N[재촬영 또는 실패 처리]
    M -- 예 --> O[빈 공간 3D pose 생성]
    O --> P[트레이 책 pose 확인]
    P --> Q[CP4 파지·삽입 경로 생성]
    Q --> R[책 파지]
    R --> S[사전 삽입 위치 이동]
    S --> T[저속 삽입·해제·후퇴]
    T --> U{배치 성공?}
    U -- 아니오 --> V[안전 후퇴 및 실패 기록]
    U -- 예 --> W[YAML 상태 갱신]
    V --> X{남은 책?}
    W --> X
    X -- 예 --> H
    X -- 아니오 --> Y[주차 위치 복귀]
    Y --> C
~~~

## 4. Critical Point

| CP | 범위 | 주 담당 | 성공 조건 |
| --- | --- | --- | --- |
| CP1 | 트레이 인수 | 이현민·김도윤 | TrayJob과 실제 트레이 정보가 일치하고 책을 파지할 수 있음 |
| CP2 | 서가 관측·작업 위치 정렬 | 이동준 | 서가가 카메라 시야와 로봇팔 작업 범위 안에 있음 |
| CP3 | 빈 공간 탐지와 pose 생성 | 윤재민 | 책이 들어갈 공간을 찾고 유효한 3D 좌표·방향·신뢰도를 생성함 |
| CP4 | 파지와 삽입 | 김도윤 | 충돌 없이 파지·삽입·해제·후퇴하고 배치를 확인함 |

### CP1 실패

트레이 ID, 책 수 또는 장착 상태가 일치하지 않으면 서가로 이동하지 않는다. 한 번 재확인한 후에도 실패하면 작업을 중단하고 오류를 기록한다.

### CP2 실패

카메라에 서가가 충분히 보이지 않거나 로봇팔 작업 범위를 벗어나면 제한된 횟수만 재정렬한다. 성공하기 전에는 비전 결과나 로봇팔 삽입을 요청하지 않는다.

### CP3 실패

빈 공간이 없거나 폭이 부족하거나 신뢰도가 낮으면 삽입하지 않는다. 한 번 재촬영한 뒤에도 실패하면 1차에서는 실패로 종료한다. 2차에서는 대체 위치와 다른 서가 후보를 탐색한다.

### CP4 실패

파지 실패 시 후퇴하고 제한된 횟수만 재시도한다. 삽입 중 충돌 또는 정렬 실패가 발생하면 전진을 멈추고 안전하게 후퇴한다. 배치 확인 전에는 점유 상태를 변경하지 않는다.

## 5. 비전 처리 흐름

~~~text
RGB-D 입력
  ↓
서가가 있을 것으로 예상되는 관측 영역 설정
  ↓
서가 전면과 선반 내부 영역 탐지
  ↓
책이 없는 깊은 영역 또는 빈 간격 추출
  ↓
후보 공간의 폭·높이·깊이 계산
  ↓
대상 책 크기 + 안전 여유와 비교
  ↓
삽입 대상 선택
  ↓
중심 좌표·삽입 방향·삽입 깊이 계산
  ↓
camera_link → arm_base_link 변환
  ↓
TargetSlot 결과 반환
~~~

## 6. 좌표계

~~~text
map
└── odom
    └── base_link
        ├── arm_base_link
        │   └── tool0
        │       └── gripper_tcp
        ├── camera_link
        │   ├── camera_color_optical_frame
        │   └── camera_depth_optical_frame
        └── tray_frame
~~~

YAML은 map 기준 서가 접근 waypoint만 제공한다. 정확한 빈 공간 pose는 camera_link에서 계산하여 arm_base_link로 변환한다. perception과 manipulation에 전달하는 pose에는 frame_id와 timestamp를 반드시 포함한다.

## 7. FSM 주요 상태

~~~text
INITIALIZING
→ IDLE
→ PLANNING
→ NAV_TO_RETURN
→ RECEIVE_TRAY
→ SELECT_BOOK
→ NAV_TO_SHELF
→ ALIGN_FOR_VISION
→ DETECT_EMPTY_SLOT
→ DETECT_TRAY_BOOK
→ PLAN_PLACE
→ GRASP_BOOK
→ INSERT_BOOK
→ VERIFY_PLACE
→ UPDATE_DATA
→ NEXT_BOOK
→ RETURN_HOME
→ COMPLETED
→ IDLE
~~~

실패 시 현재 동작을 안전하게 멈춘 뒤 RETRY, WAIT_FOR_OPERATOR 또는 FAILED로 전이한다. 정확한 재시도 횟수는 system.yaml에서 관리한다.

