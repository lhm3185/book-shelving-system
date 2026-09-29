# 브랜치 정리 (2026-09-29, 도윤님 결정)

## 지금 있는 것

| 브랜치 | 무엇 | 누구 |
|---|---|---|
| `feature/robot_control` | 로봇팔 1차 작업 (마지막 커밋 09-22 16:14). 그대로 둔다 | 김도윤 |
| **`feature/grasp_advanced`** | **그 뒤의 고도화 전부** — 서가 둘 · 네 권 · 스캔 한 번 · 새 레벨 · 교육장 판. 옛 `work/demo_0928` 의 끝(d6f0348)에서 시작 | 김도윤 |
| `integration/manipulation-standalone` | 제출용 통합본 (AMR + 로봇팔, 책 한 권) | 이현민 |
| `feature/1st_combine_integration_test` | 1차 시연 | 팀 |
| `main` · `vision` · `refactor/project-layout` | 손대지 않았다 | 팀 |

옛 문서와 로그에 나오는 `work/demo_0928` 은 **`feature/grasp_advanced`** 로 읽는다. 커밋 해시와 동결 태그(`freeze-…`)는 그대로다.

## 지운 브랜치와 보관 태그

지우기 전에 각 브랜치의 끝에 `archive/<브랜치 이름>` 태그를 달았다. 원격에서 태그 = 브랜치 끝임을 확인한 뒤 지웠다.

| 지운 브랜치 | 보관 태그가 가리키는 커밋 | `feature/grasp_advanced` 에 없는 것 |
|---|---|---|
| `feature/amr_patrol_pickplace` | ccf755a | 없음 |
| `work/grasp_refine_0923` | 5b03028 | 없음 |
| `work/motion_api_0924` | fac2ead | 없음 |
| `work/vision_merge_0924` | 0eae70e | 없음 |
| `work/scan_once_0928` | e094a5d | 없음 |
| `work/rnd_0925` | b439f14 | 커밋 2개 — 같은 내용이 이미 들어 있다 |
| `work/night_0929` | 3c3a55b | 커밋 2개 — 같은 내용이 이미 들어 있다 |
| `test/vision-slot` | 6b8c289 | 병합 커밋 2개 + 같은 내용 1개 |
| `work/record_cams_0928` | 02cbb97 | **커밋 2개가 다른 곳에 없다** (레벨 카메라 녹화 `REC_CAMS` · `frames.csv`) |
| `work/demo_0928` | d6f0348 | 없음 (`feature/grasp_advanced` 의 출발점) |

`work/record_cams_0928` 의 두 커밋은 옮기지 **않았다.** 체리픽하면 검증된 `run_simulation.py` · `make_clips.sh` 와 충돌한다
(그 뒤 녹화는 `SIM_REC_AUTO` 로 다시 만들었다). 발표 전날 검증된 실행 파일을 손으로 합치지 않기로 했다. 필요하면 보관 태그에서 꺼낸다.

## 되살리는 법

```bash
git fetch origin --tags
git push origin 'archive/work/rnd_0925^{commit}:refs/heads/work/rnd_0925'     # 브랜치를 원격에 다시 만든다
git switch -c work/rnd_0925 archive/work/rnd_0925                             # 내 PC 에서만 볼 때
```
