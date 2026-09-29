# 새 레벨 (촬영용 셰이딩) — integration/manipulation-standalone 에 끼우는 꾸러미

만든 날: 2026-09-29 · 만든 이: 김도윤(로봇팔) · 받는 이: 통합 담당자

## 한 줄 요약

**폴더 하나를 바꾸고 설정 한 줄을 고치면 된다.** 통합 브랜치 커밋 `3a3492e` 에 이대로 넣어 책 한 권 전체 사이클이 끝까지 돌았다
(10.10.0.1 `IsaacSim15`, 2026-09-29 12:36 ~ 12:44).

## 들어 있는 것

```
worlds/library/library_environment.usd     환경 본체 (16.6 MB)  md5 40dae43e00e597f7114084f547eb800a
worlds/library/env_base_collider.usd       방 껍데기 충돌체 — 본체가 상대 경로로 참조한다
worlds/library/textures/                   텍스처 92개 (215 MB)
scenes.yaml.patch                          설정 한 줄
MD5SUMS                                    전체 파일 확인용
evidence/                                  돌린 판의 로그와 화면
```
폴더 밖이나 인터넷을 가리키는 참조는 없다(`UsdUtils.ComputeAllDependencies` 로 확인 — 레이어 2개, 에셋 92개, 못 찾는 것 0).

## 넣는 법

```bash
cd ~/book-shelving-system
mv isaac_sim/assets/worlds/library isaac_sim/assets/worlds/library.orig      # 옛 것을 옆에 둔다
cp -a <이 꾸러미>/worlds/library isaac_sim/assets/worlds/library
git apply <이 꾸러미>/scenes.yaml.patch                                       # 또는 아래 한 줄을 손으로
(cd <이 꾸러미> && md5sum -c MD5SUMS)                                         # 복사 확인
```

`isaac_sim/config/scenes.yaml` 의 한 줄:
```
shelf_prim_path: /World/bookshelves/shelf_brown__book_shelf_01
→
shelf_prim_path: /World/bookshelves_main/shelf_brown__book_shelf_11
```
**이 줄을 안 고치면 서가를 못 찾는다.** 새 레벨은 작업 서가가 `/World/bookshelves_main` 아래에 있고, 같은 자리의 서가 번호가 다르다
(통합 월드의 `_01` 자리 = 새 레벨의 `_11`, 월드 x 1.87 ~ 3.28 · y −2.57 ~ −2.27).

그 밖에 고칠 것은 없다 — `world_usd` 경로 · 지도 · 주행 목표 · 트레이 · 로봇은 그대로다.

## 원래 레벨과 무엇이 같고 다른가

| | 통합 브랜치의 월드 | 이 꾸러미 |
|---|---|---|
| 방 크기 · 서가 16개의 자리 · 반납기 자리 | — | **같다** (bbox 를 0.01 m 단위로 비교) |
| 작업 서가 | `/World/bookshelves/…_01` | `/World/bookshelves_main/…_11` |
| 나머지 서가 | `/World/bookshelves/*` | `/World/bookshelves_Prop/*` (장식, 14개) |
| 서가 책 | `/World/books/<층>/<책>` | `/World/bookshelves_main/books/shelf_{A,B}/<층>/<책>` — 팔 코드(`shelf_gap.py`)가 둘 다 읽는다 |
| 반납기 | `/World/ReturnMachine` | `/World/return_machine_final` (새 모델, 높이 1.61 → 1.73 m) |
| 바닥 · 벽 | `/World/floor` | `/Environment/Floor` · `/Environment/Wall` (재셰이딩) |
| 조명 | RectLight 5000 · Distant 1000 | RectLight 8500 (12 m × 10 m, 높이 6 m) · Distant 1500 |
| 로봇 · 트레이 · 트레이 책 | 없음 (따로 얹는다) | **없음** — 우리 레벨에서 뺐다 |

## 우리 레벨에서 바꾼 것 (통합 브랜치에 맞추려고)

원본은 로봇팔 고도화 작업에 쓰던 `Final_Level_Shaded_robot` (조명을 올린 판)이다. 거기서:

1. **`/World/ridgeback_franka` 를 뺐다** — 통합 브랜치는 `franka_nav.usda` 를 따로 얹는다. 안 빼면 같은 경로에 로봇 둘이 겹친다.
2. **`/World/tray_books` 를 뺐다** — `tray_set.usd` 가 같은 경로에 트레이와 책을 얹는다.
3. **트레이 책 머티리얼 7개를 뺐다** — `tray_set.usd` 와 이름이 겹친다. 환경에서 쓰는 곳은 없었다.
4. **방 껍데기(`/World/env_Base/Cube_002`) 충돌체를 `convexDecomposition` → `none` 으로.** 통합 월드와 같은 값이다.
   **이것을 안 하면 로봇이 출발 자리에서 못 움직인다**(아래).

## 돌려 본 것 (10.10.0.1, 통합 브랜치 3a3492e, 책 한 권)

| 판 | 월드 | 결과 |
|---|---|---|
| 12:19 | 통합 브랜치 원래 월드 (기준선) | **완주** — 작업 발행 12:20:35 → `COMPLETED -> IDLE` 12:28:02 (7분 27초) |
| 12:32 | 이 꾸러미, 4번을 하기 전 | **출발 자리에서 멈춤** — `Controller patience exceeded` 14건, `cmd_vel` 0, 오도메트리 속도가 2.3 m/s 로 튄다 |
| 12:36 | **이 꾸러미** | **완주** — 작업 발행 12:37:46 → `COMPLETED -> IDLE` 12:43:41 (5분 55초), 배치 검증 다섯 항목 전부 true, 주행 오류 0건 |

12:36 판의 배치 검증: `upright: true · depth: true · spine: true · x: true · floor: true` · `fallen_books: []`.
`Failed to get result for follow_path in node halt!` 가 한 번 나왔다 — 기준선에서도 두 번 나온 줄이고 README 가 무시해도 된다고 적은 줄이다.

## 알아 둘 것

- **시험은 ROS 도메인 77 로 했다.** 옆 PC 에서 AMR 주행 시험이 돌고 있어 팀 도메인 130 을 피했다. 시험본에서만
  `scenes.yaml` 의 `domain_id` 와 `run_standalone.sh` 의 `ROS_DOMAIN_ID` 를 77 로 바꿨다. **이 꾸러미에는 그 변경이 없다.**
- **시험 PC 의 파이썬은 프로젝트 `.venv` 가 아니다.** 시스템에 깔린 ultralytics 8.4.153 · numpy 1.26.4 를 `.venv` 자리에 이어 썼다
  (README 의 8.4.155 와 0.0.2 차이).
- **표본은 한 판이다.**
- 저장소에 올릴 때: `.usd` · `.png` 는 LFS 다. 텍스처 92개 중 87개는 통합 브랜치에 이미 같은 내용이 있고(해시 일치), 새로 올라가는 것은
  바닥 · 벽 텍스처 5개 10 MB 와 본체 16.6 MB 다. 이름이 같은데 내용이 다른 텍스처는 없다.
- Isaac 창에 `ScriptNode Warning` 대화상자가 뜬다. 로봇 층의 주행 그래프 때문이고 원래 월드에서도 같다.
- 트레이에는 `tray_set.usd` 그대로 책 다섯 권이 실린다. 한 권만 집어 꽂는 것은 통합 브랜치의 의도된 범위다.
