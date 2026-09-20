# 시연 실행 — FSM 담당자용 4개 명령

> 로봇팔(김도윤) 쪽에서 준비한 진입점입니다. **추가 기능은 없고, 기존 실행을 감싼 것**입니다.
> 상세 배경은 `docs/doyoon-kim/manipulation/DEMO_20260921.md`.

---

## 한눈에

```bash
# 터미널 A — Isaac (GPU PC 로 ssh 는 스크립트가 알아서 한다. 이 창은 계속 열어 둔다)
PROFILE=pick ./scripts/demo/sim_up.sh

# 터미널 B — 이 PC 의 ROS 노드 (A 에 '준비 완료' 가 뜬 뒤. 이 창도 계속 열어 둔다)
./scripts/demo/nodes_up.sh

# 터미널 C — 연결 확인 → 명령
./scripts/demo/status.sh
./scripts/demo/pick.sh
```

터미널 A·B 는 **닫으면 죽습니다.** C 만 반복해서 씁니다.

---

## 처음 한 번만

```bash
cp scripts/demo/demo_local.env.example scripts/demo/demo_local.env
```

GPU PC 주소·경로가 다르면 이 파일만 고칩니다 (git 에 안 올라갑니다).

---

## 프로파일 — 레벨과 좌표는 **한 쌍**입니다

| PROFILE | 무엇 | 상태 |
| --- | --- | --- |
| `pick` (기본) | 트레이 책을 **집어 드는 것까지** | **2026-09-20 재현 확인** |
| `shelf` | 실제 서가 앞에서 **꽂기까지** | **미해결** — `approach` 가 어깨 막힘으로 시간 초과 |

레벨 USD 와 꽂을 좌표를 따로 넘기지 않게 묶어 두었습니다.
**이걸 따로 넘기면 반드시 섞입니다** — 9/20 에 Franka 좌표(z 0.3399)를 M0609 레벨에 써서
두 선반 단 사이 허공을 가리킨 적이 있습니다.

```bash
PROFILE=shelf ./scripts/demo/sim_up.sh     # 레벨·서가 prim·선반 높이가 함께 바뀐다
PROFILE=shelf ./scripts/demo/pick.sh       # 좌표도 같이 바뀐다
```

---

## 각 명령

### `sim_up.sh` — Isaac 띄우기

```bash
PROFILE=pick ./scripts/demo/sim_up.sh
SIM_HOST=local ./scripts/demo/sim_up.sh    # GPU PC 에 직접 앉아서 할 때
DRY_RUN=1 ./scripts/demo/sim_up.sh         # 실행 안 하고 명령만 본다
```

약 4분 뒤 이 두 줄이 나오면 준비 완료입니다:

```
### 시작 자세를 홈으로 고정 (오차 0.0012 rad)
### 준비 완료 (step 3) — 명령 대기 /manipulation/sim/command
```

### `nodes_up.sh` — 비전 + 로봇팔 노드

```bash
./scripts/demo/nodes_up.sh
SHOW_VISION=1 ./scripts/demo/nodes_up.sh   # 비전 CV 창(바운딩박스)까지
```

### `status.sh` — 보내기 전에 확인

`place_book` 이 안 보이면 **여기서 멈추고** 원인을 잡습니다. 안내 문구가 같이 나옵니다.

### `pick.sh` — 책 한 권

```bash
./scripts/demo/pick.sh
BOOK_ID=book_1 ./scripts/demo/pick.sh
GOAL_X=-0.3897 ./scripts/demo/pick.sh      # 같은 프로파일에서 칸만 바꿀 때
FEEDBACK=1 ./scripts/demo/pick.sh          # 단계별 진행 보기
```

---

## FSM 노드에서 직접 부를 때

스크립트를 거치지 않고 액션을 직접 보내도 됩니다. 계약은 그대로입니다:

| 필드 | 값 |
| --- | --- |
| 액션 | `/place_book` (`shelving_interfaces/action/PlaceBook`) |
| `target_slot.header.frame_id` | **`arm_base_link`** (다른 값은 410 으로 거절) |
| `target_slot.pose.position` | **꽂힌 뒤 책의 AABB 중심** (손끝 위치 아님) |
| `target_slot.pose.orientation` | yaw +90° → `z=w=0.7071068` |
| `job_id`, `book_id` | 비어 있으면 거절 |

> **`base_link` 를 쓰면 안 됩니다.** 결합 로봇에서 `base_link` 는 AMR 몸체라
> 팔 기준과 **0.655 m** 어긋납니다.

오류 코드는 `docs/doyoon-kim/web_claude/learning_manipulation_node.md` §3 참조.
요약: **410 = 요청이 잘못됨(재시도 금지)**, 411 = 아직 준비 안 됨(잠시 뒤 재시도),
404/405/406/407/409 = 파지·삽입 실패(다음 책으로).

---

## 알려진 제약 (2026-09-20 기준)

| | |
| --- | --- |
| `pick` 프로파일 | 접근·파지까지 확인. **운반 중 M406**(손 안에서 책 미끄러짐)이 날 수 있음 — 기존 이슈 |
| `shelf` 프로파일 | **아직 통과 못 함.** 배제 목록은 `docs/doyoon-kim/web_claude/v20.md` §5 |
| 한 세션 6권 | 트레이 책은 Isaac 한 세션에 6권. 더 하려면 재시작 |
| 트레이 5·6번 칸 | 신뢰도 낮음. 0~3번 칸 권장 |
| Isaac 재시작 | **`nodes_up.sh` 도 반드시 다시** |
| 도메인 | 터미널·ssh 를 넘지 않음. 전부 `130` |

---

## 검증 상태 (2026-09-21 새벽 갱신)

**GPU PC 에 접속할 수 없어 실제 Isaac 기동까지는 못 돌려 봤습니다.** 그 아래는 전부 확인했습니다.

| 항목 | 상태 |
| --- | --- |
| bash 문법 (`bash -n`) 전 파일 | 확인 |
| `DRY_RUN=1` 생성 명령 (pick/shelf 둘 다) | 확인 |
| **원격에 실릴 스크립트를 `bash -n` 으로 재검사** | 확인 |
| base64 전송 왕복 일치 (한글·따옴표 포함) | 확인 |
| 환경변수 우선순위 (`GOAL_X=...`, `SIM_HOST=local`) | 확인 |
| DDS 화이트리스트 가드 3경로 (막힘/해제/강제) | 확인 |
| 모델 동봉본 대체 | 확인 (없는 경로를 줘서) |
| `status.sh` 실패 판정 | 확인 |
| 감싸는 대상 명령 자체 | 9/20 낮 GPU PC 에서 동작 확인 |
| **Isaac 기동 전 구간** | **미검증** |

### 독립 점검에서 12건을 고쳤습니다 (2026-09-21)

제3자 시선으로 점검받아 **조용히 틀리는 버그** 를 잡았습니다. 대표적인 것:

- 프로파일이 환경변수를 덮어써서 `GOAL_X=... ./pick.sh` 가 **무시**됐습니다.
  칸을 바꿨다고 생각하면서 팔은 이전 동작을 반복하고, **출력까지 덮어쓴 값을 찍어**
  화면과 동작이 일치해 보였습니다
- `_common.sh` 가 DDS 화이트리스트 해제 경로를 무력화해, 연구실 밖에서 `nodes_up.sh` 가
  통째로 막히고 빠져나갈 방법이 없었습니다
- `status.sh` 가 파이프라인 종료코드를 봐서 **시뮬이 죽어도 "정상"** 으로 보고했습니다

전부 재현한 뒤 고쳤고, 고친 뒤 다시 재현해 확인했습니다.

문제가 생기면 `DRY_RUN=1 PROFILE=... ./scripts/demo/sim_up.sh` 로 나온 명령을
GPU PC 에서 직접 붙여넣어 보면 스크립트 문제인지 아닌지 바로 갈립니다.
