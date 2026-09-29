# 교육장 이식 체크리스트 — 집 데스크탑 → 교육장 GPU PC

김도윤(D) · 2026-09-28 밤 · 9/29 교육장 시연 준비, 9/30 최종 발표
작성: 노트북 세션. 데스크탑 세션의 이식성 보고(심볼릭 링크 2 · 원격 payload · 블렌더 상대경로)를 받아 정리.

> **시연은 무조건 새 레벨로 한다** (도윤님 결정, 2026-09-29). 옛 레벨은 기준선 판으로만 쓴다.
>
> **원칙 셋.** ① 교육장에 인터넷이 없다고 가정한다. ② 옛 레벨은 "PC 문제인가 레벨 문제인가" 를 가르는 기준선으로 가져간다.
> ③ 현장 PC 에서 회귀 판이 서기 전에는 무대에 서지 않는다(`LIVE_DEMO_RUNBOOK.md` §1.3).

## 0. 교육장 GPU PC — 아는 것과 모르는 것

| 항목 | 값 | 근거 · 시점 |
|---|---|---|
| 주소 · 계정 | 10.10.0.2 · rokey | 메모 (9/23) |
| GPU | NVIDIA GeForce RTX 5080 Laptop, VRAM 16 GB | Isaac 기동 로그 (9/17) |
| CPU · RAM | Core Ultra 9 275HX 24코어 · 64 GB | 같은 로그 |
| OS · 커널 | Ubuntu 24.04.4 · 6.14.0-27 | 같은 로그 |
| NVIDIA 드라이버 | 580.173.02 | 같은 로그 |
| Isaac Sim | 5.1, `~/isaacsim` | 같은 로그 (Kit 5.1 경로) |
| ROS 2 | Jazzy | 9/17~9/24 실행 기록 |
| 디스플레이 | `:1` (`:0` 으로 띄우면 창이 안 보인 채 돈다) | 메모 (9/22) |
| ROS 도메인 | 팀 130 + FastDDS 화이트리스트. 우리 시연은 129 + `config/fastdds_local.xml` | 메모 |
| 사용 | 팀원과 공용. 밤 10시경 전원 끔. 남의 프로세스 종료 금지 | 메모 |
| **인터넷** | **모름** — 없다고 가정 | 미확인 |
| **로봇 에셋 캐시** | **모름** — 9/24 까지 같은 로봇을 띄웠으니 있을 수 있으나 믿지 않는다 | 미확인 |
| **디스크 여유** | **모름** | 미확인 |

전부 9/17 ~ 9/24 시점 값이다. 현장에서 1단계로 다시 확인한다.

## 1. 현장 도착 후 가장 먼저 (판을 태우기 전, 5분)

```bash
who; nvidia-smi --query-compute-apps=pid,used_memory,process_name --format=csv   # 누가 쓰는 중인가
nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv
ls /tmp/.X11-unix/                       # DISPLAY 번호
ls ~/isaacsim/VERSION && cat ~/isaacsim/VERSION
df -h ~ | tail -1                        # 꾸러미 들어갈 자리
du -sh ~/.cache/ov 2>/dev/null           # 에셋 캐시 유무 (있어도 의존하지 않는다)
timeout 5 curl -sI https://omniverse-content-production.s3-us-west-2.amazonaws.com/ | head -1   # 인터넷
```

## 2. 꾸러미 — 집에서 만들어 USB 로 가져간다

```
b1_demo_bundle/
  level_shaded/        새 레벨. Isaac "Collect" 로 모은 폴더 (의존 파일 전부, 상대경로)
  level_rnd/           옛 레벨(Final_Level_RnD) — **기준선 판 전용.** 시연에는 쓰지 않는다
  assets/              책 usd 넷 · 트레이 usd
  repo/                저장소 (태그 freeze-20260928-scan-once)
  weights/             비전 모델 가중치 (비전 노드가 읽는 경로 그대로)
  demo_env.sh          이 PC 의 절대경로를 한 곳에서 정하는 환경변수 파일 (저장소 밖)
  video/               집에서 찍은 완주 영상 — 현장이 안 서면 이걸 튼다
  MD5SUMS  README.txt
```

### 2.1 레벨을 스스로 서게 만든다

1. **심볼릭 링크 둘을 실파일로.** `franka_camera.usd`, `_rnd_ref.usd`.
2. **`env_Base` 를 작은 파일로 떼어낸다.** `_rnd_ref.usd` 를 통째로 참조하면 Collect 가 옛 레벨과 그 의존 파일
   전부를 끌고 온다. 옛 레벨에서 `/World/env_Base` 하위만 `Sdf.CopySpec` 으로 `env_base_collider.usd` 에 복사하고
   새 레벨은 그 파일을 참조한다. **손으로 다시 쓰지 않는다** — 검증된 형상을 그대로 옮긴다.
   옮긴 뒤 판별 지표(팔 베이스 ↔ 루트 0.300 m)를 다시 잰다.
3. **원격 의존 셋을 로컬로.** `franka_camera.usd` 는 원격 주소를 셋 가리킨다:
   ```
   …/Isaac/5.1/Isaac/Robots/Clearpath/RidgebackFranka/ridgeback_franka.usd   (payload)
   …/Isaac/5.1/Isaac/Sensors/Intel/RealSense/rsd455.usd                       (payload)
   …/Isaac/5.1/Isaac/Sensors/NVIDIA/Example_Rotary.usda                       (reference, 라이다)
   ```
   각 파일은 다시 메시·재질·텍스처를 가리킨다. 하나씩 받지 말고 Isaac 의 **Collect**(File → Collect As,
   `omni.kit.tool.collect`)로 레벨째 모은다. 옛 레벨도 `Collected_ing_library_env_v5/` 로 그렇게 전달받은 전례가 있다.
4. **블렌더 파일**은 시연에 필요 없다. 가져가려면 `File → External Data → Pack Resources`.

### 2.2 모은 뒤 검사 — 캐시를 건드리지 않고

```python
# Isaac 파이썬 또는 pxr 가 있는 파이썬에서
from pxr import UsdUtils, Sdf
layers, assets, unresolved = UsdUtils.ComputeAllDependencies(Sdf.AssetPath("level_shaded/<레벨>.usd"))
bad = [p for p in [l.identifier for l in layers] + list(assets)
       if p.startswith("http") or "/home/" in p and "b1_demo_bundle" not in p]
print("못 찾은 것", unresolved); print("꾸러미 밖을 가리키는 것", bad)   # 둘 다 비어야 한다
```
그리고 `grep -rl "https://" level_shaded/ | head` 가 비어야 한다(usda 기준. usdc 는 위 스크립트로).

### 2.3 저장소에 넣지 않는 것 — 라이선스 확인 결과 (2026-09-28 밤)

NVIDIA 공식 문서(License FAQ)에서 확인한 것:

- NVIDIA 가 제공하는 에셋(로봇·센서·환경 USD)은 **"NVIDIA Isaac Sim Additional Software and Materials License"** 아래에 있다.
  Isaac Sim 소스 코드의 라이선스와는 별개다.
- 문서 원문: 추가 구성요소(Omniverse Kit SDK, 에셋 등)는 "라이선스가 명시적으로 허용하는 경우를 빼고
  **수정하거나 재배포할 수 없다**".
- 오프라인 사용은 허용된다. 에셋 묶음(Robots & Sensors 등)을 내려받아 쓰는 절차가 공식 문서에 있고,
  "온라인이든 오프라인이든 같은 조건" 이라고 적혀 있다.

**결론**
- 로봇·센서 에셋을 **공개 저장소에 커밋하지 않는다.** 재배포에 해당한다.
- 꾸러미(USB)로 우리 PC 사이에서 옮겨 Isaac 을 돌리는 것은 오프라인 사용이다.
- 라이선스 전문은 읽지 못했다(FAQ 와 요약만 확인). 저장소 공개 범위를 바꾸거나 에셋을 넣을 일이 생기면 전문을 먼저 읽는다.
- Collect 로 모은 폴더는 `.gitignore` 에 넣는다.

**다른 길 하나**: 교육장 PC 에 인터넷이 있으면 공식 에셋 묶음을 받아
`--/persistent/isaac/asset_root/default=<경로>` 로 가리킬 수 있다. 다만 우리 레벨은 에셋을 **절대 주소(https)** 로
가리키므로 이 설정만으로는 안 바뀐다 — 레벨 쪽 경로를 고치거나 Collect 가 여전히 필요하다.

출처: NVIDIA Isaac Sim 문서 License FAQ · Setup Tips(로컬 에셋 묶음).

## 3. 현장에서 바뀌는 것 — 미리 알고 간다

| 바뀌는 것 | 왜 문제인가 | 대응 |
|---|---|---|
| **첫 기동 셰이더 컴파일** | 새 PC · 새 레벨 첫 로드는 수 분 걸린다. 그 사이 노드가 시간 초과 | 전날 또는 당일 아침 **한 번 띄워 데운다**. T−30분으로는 부족할 수 있다 |
| GPU 가 다르다 (4080 SUPER → 5080 Laptop) | 시뮬 속도가 달라 권당 시간이 바뀐다. `goal_timeout_s` 300 대비 여유 재확인 | 회귀 판에서 권당 시간을 적는다 (집 115 ~ 121 s) |
| 절대경로 | `full_cycle.sh` 기본값이 `/home/rokey/b1_work/…`, `/home/rokey/env_v5/…` | `demo_env.sh` 에서 `SIM_LEVEL` · `SIM_BOOKS` · 서가 prim 을 명시 |
| DISPLAY | `:1` | `demo_env.sh` 에 넣는다 |
| ROS 도메인 · DDS | 팀 화이트리스트가 같은 PC 안 디스커버리를 막는다 | 도메인 129 + `config/fastdds_local.xml` |
| 공용 PC | 팀원 프로세스와 GPU 메모리가 겹친다 | 1단계에서 확인. 남의 것은 건드리지 않는다 |
| 조명 · 새 바닥 | 트레이 검출 여유가 원래 0 이었다 | 새 레벨 회귀 판에서 관측 수를 옛 레벨과 나란히 |

## 4. 현장 관문 — 순서

**2026-09-29 갱신**: **쓰는 태그는 `freeze-20260929-shaded-r2` (86ebe79)** — 트레이 네 권 레벨. 집 데스크탑에서 새 셸로
`source …/demo_env.sh` → `bash scripts/demo/full_cycle.sh` 두 줄에 두 판 연속 4/4. r1 은 다섯 권 레벨의 태그,
r 없는 태그는 시작하자마자 죽으니 쓰지 않는다. 아침 판이 읽은 것 전부와 그 위치는 `cli_exchange/handoff_0929/SITE_MANIFEST.md`.

**교육장 도착 확인 (09:04)**: GPU PC 10.10.0.2 연결됨 · Isaac 5.1.0 · 디스플레이 `:1` · GPU 비어 있음 · **인터넷 됨** ·
에셋 캐시 12 GB · 디스크 760 GB · 비전 파이썬 패키지는 집 데스크탑과 사실상 같음. 꾸러미는 `~/b1_demo_bundle` 에 복사했고
md5 232/232. **`~/b1_arm` 에는 옛 브랜치(night/0922)와 커밋 안 된 수정 둘이 있다 — 건드리지 않고 새 폴더에 받는다.**
로봇 에셋은 하나씩 옮기지 못한다 — `ridgeback_franka.usd` 가 다시 13개, `rsd455.usd` 가 5개를 참조한다(2026-09-29 실측).
Collect 본이 없으면 인터넷이 필수다.

```
① 1단계 확인 (5분)
② 옛 레벨 회귀 1판                검증된 조합이 이 PC 에서 서는가 — 기준선
③ 새 레벨 회귀 1판                판별 지표 0.300 m · 로봇 홈 (+4.986, −5.607) yaw +90.00° · 네 권
④ 새 레벨 2판 더                   3/3 이면 시연 준비 끝. 안 서면 **새 레벨 안에서** 원인을 푼다 (옛 레벨로 접지 않는다)
⑤ LIVE_DEMO_RUNBOOK 의 초록불 셋
```
②가 서기 전에 ③을 돌리지 않는다. ②가 안 서면 레벨이 아니라 PC 문제다.

## 5. 도윤님 결정이 필요한 것

- ~~시연 레벨~~ — **정해졌다. 무조건 새 레벨**(2026-09-29).
- **`LIVE_DEMO_RUNBOOK.md` 갱신.** 절차서는 두 권 · "매 판 다시 스캔" 기준이다. 네 권 · 스캔 한 번 기준으로 고쳐야 한다.
- **영상을 기본으로 둘지.** 현장 생중계가 필수가 아니면 집에서 찍은 완주 영상이 가장 안전하다.
