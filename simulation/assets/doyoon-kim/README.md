# D 김도윤 — 시뮬레이션 에셋

에셋 제작·모델링 담당 산출물. 모든 USD 는 **같은 폴더의 `textures/` 를 상대경로로 참조**한다 → 폴더째로 옮길 것.
원본 `.blend` 는 LFS 대상(`.gitattributes`)이 아니라 저장소에 넣지 않았다 (필요하면 LFS 대상 추가를 이현민과 합의).

| 폴더 | 파일 | 내용 | Isaac 확인 (2026-09-17) |
| --- | --- | --- | --- |
| `tray/` | `tray_v1.usdc` | 책 6권 트레이 (칸 간격 0.075 m, 칸막이 0.01·높이 0.10). Blender 파라메트릭 제작 (`isaac_sim/isaac/make_tray.py`), 라이트 없음 | 책 6권 세워 여유 8.2 mm (`tray_v1_fit.jpg`), 트레이 4권 연속 파지·꽂기 4/4 |
| `shelf/` | `shelf_darkbrown_filled.usda` (+ `__book_shelf.usdc`, `__book_cube.usdc`) | **배경용** 책이 찬 서가 1.41×0.31×1.80 m, 짙은 갈색 금속성 목재(0.8) | UV·텍스처 정상, 앞뒤 책 글자 방향 정상 (`shelf_darkbrown_filled_isaac_check.jpg`). 충돌체 없음 |
| `shelf/` | `shelf_brown__book_shelf.usdc` | 개방형 갈색 서가 (비전 학습 데이터와 같은 에셋) | |
| `env_wall_floor/` | `wall_01~06.usdc`, `floor_01.usdc`, `library_floor_v002.usdc` | 도서관 벽·바닥 (원본 익스포트) | **`library_floor_v002.usdc` 는 쓰지 말 것**: 노말맵이 Normal Map 노드 없이 연결·sRGB, SphereLight·DomeLight·Camera 포함 |
| `env_wall_floor/usd_fixed/` | `library_floor_v002__library_floor.usdc` | 위 바닥 수정본: 노말 scale 2·bias −1·raw, UV `st`, 조명·카메라 제외. 2×2 m | 원본 대비 표면 요철 정상 (`isaac_compare_left_orig_right_fixed.jpg`, 왼쪽 원본) |

`simulation/assets/tray.usd`, `book.usd` 자리표시 파일은 레벨 통합(이현민) 때 위 파일로 교체할지 정한다.
