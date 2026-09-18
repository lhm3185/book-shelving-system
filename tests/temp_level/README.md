# 도서관 레벨 v3 / v4 (2026-09-17)

## 넣는 위치 — 책과 같은 규칙 (레벨 기준 상대경로)
```
~/Desktop/ing_library_env_v3.usd        (또는 v4)
~/book_dataset/usd_v2/...               책 (이미 있음)
~/book_dataset/dataset/env_wall_floor/  바닥 (이미 있음)
~/book_dataset/assets/shelf/usd/        ← 이 폴더의 book_dataset/assets/shelf/usd 를 그대로 복사
~/book_dataset/assets/franka_camera.usd ← v4 만 필요 (카메라·라이다 탑재 로봇)
```
선반 파일은 저장소 `simulation/assets/doyoon-kim/shelf/` 와 같은 파일이다.

## 바뀐 점
- 선반 16개 경로: `../Downloads/shelf-20260916T122706Z-1-001/...` (GPU PC 전용 폴더라 다른 PC 에서 안 보였음) → `../book_dataset/assets/shelf/usd/...`
- `shelf_brown__book_shelf_01` (x 2.57, y −2.42, 로봇 파지·꽂기 시험 선반): 갈색 빈 선반 유지
- 나머지 15개: 책이 찬 배경용 선반(`shelf_darkbrown_filled.usda`)으로 교체. 위치·z×1.4 크기 유지, 충돌체 없음
- v4 = v3 + 로봇을 `franka_camera.usd` 로 교체
