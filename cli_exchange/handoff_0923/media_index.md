# 영상·사진 목록 (맨 위 = before/after)
| 파일 | 장면 | 볼 프레임 | 한 줄 |
| --- | --- | --- | --- |
| (before) 영상 없음 — v02_default/f1_regress 는 녹화 안 함 | 기본 스위치 | – | 406/409, 책 바닥 (로그만) |
| (after) v14_side_grid.jpg (night/runs/v14_rec_side/frames 237장) | demo_best 측면 | f120~f132 트레이 벽 위 통과 · f140 reorient 중 책 45° 기움 · f156~f236 책이 맨 아래 선반판 위에 섬 | 성공기준 5·6·7·8 |
| (after) v15_rear_grid.jpg (night/runs/v15_rec_rear/frames 229장, 균등 8장 #0 #32 #65 #97 #130 #162 #195 #228) | demo_best 뒤쪽 | #97 파지 · #130 서가로 뻗음 · #162·#195 손이 트레이 위에서 책을 세로로 든 채 솟음(reorient d) · #228 반납 | 솟음 흠 확인, 꽂힌 책은 손에 가림 |

## v09_rec_rear — 스윙 조합 성공 판 (success=True code=0 verified=True), 뒤쪽 시점, 246프레임 (night/runs/v09_rec_rear/frames)
- 조합: SIM_PICK_Y=-3.019 SIM_RELEASE_OPEN_FIRST=1 SIM_CARRY_MODE=swing SIM_RETURN_MODE=swing (+SIM_SWING_MIRROR 미커밋 코드, 기본 켜짐)
- v09_rec_rear_sheet16.png (16등분), v09_rec_rear_key6.png (#114 #131 #163 #196 #212 #245)
- 보이는 것: #65 도착 · #82~98 트레이에서 빨간 책 파지 · #114 들어 올림 · #131 서가 가운데 선반 높이로 뻗음 ·
  **#147~180, #212 손이 트레이 위에서 위로 세워져 책을 세로로 든 채 솟음 (경로검사 carry z최대 0.755, 상한 +0.130 과 맞음)** ·
  #196, #229~245 서가 선반으로 뻗음. 서가↔트레이 위를 왕복하는 모양.
- 안 보이는 것: 마지막에 책이 선반에 꽂혀 있는지 — 이 각도에서는 손에 가려 확인 불가. 측면·정면 필요.
- 다음 녹화 때: 단계별(phase) 시각을 프레임 번호에 맞춰 적을 것 — 지금은 어느 프레임이 carry/wedge/return 인지 로그와 대조가 안 된다.
