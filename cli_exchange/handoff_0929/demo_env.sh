#!/usr/bin/env bash
# ┌──────────────────────────────────────────────────────────────────────┐
# │ 저장소 사본 — 원본은 데스크탑의 ~/b1_demo_bundle/demo_env.sh 다.      │
# │ 교육장에서는 `cp cli_exchange/handoff_0929/demo_env.sh ~/b1_demo_bundle/`│
# │ 로 놓고 **아래 [고칠 곳] 세 줄만** 이 PC 에 맞춘다.                   │
# │                                                                      │
# │ [고칠 곳]                                                            │
# │   1. BUNDLE          — Collect 꾸러미를 푼 자리                       │
# │   2. FASTRTPS_…FILE  — 저장소를 받은 자리 ($HOME/b1_arm 이 아니면)     │
# │   3. DISPLAY         — 교육장은 :1 이어야 창이 보인다 (아래 주의 참고) │
# │                                                                      │
# │ 나머지는 `freeze-20260929-shaded` 태그가 재현한 값이라 건드리지 않는다.│
# └──────────────────────────────────────────────────────────────────────┘
# b1 시연 환경 — 데스크탑(집)에서 실제로 쓰는 값. 저장소 **밖**에 둔다(동결 코드를 안 건드린다).
#   source demo_env.sh && cd ~/b1_arm && bash scripts/demo/full_cycle.sh
#
# 교육장에서는 아래 두 줄만 바꾸면 된다 (Collect 꾸러미를 푼 자리).
BUNDLE="${BUNDLE:-$HOME/b1_demo_bundle}"
export SIM_LEVEL="${SIM_LEVEL:-$BUNDLE/level_shaded/Final_Level_Shaded_robot.usdc}"

# ── 유령 검출 차단 (2026-09-29 실측) ─────────────────────────────────
#   비전이 트레이 주변의 고정 물체(추정: 빈 칸 칸막이·테두리)를 책으로 본다.
#   진짜 책 x -0.365~-0.313 · 유령 x -0.452. x 하한 -0.40 이 유일하게 깨끗한 축이다
#   (y 로는 못 가른다 — 유령이 책 칸 사이에 있다). 진짜 책 35 mm 여유.
export VISION_ROI_MIN="[-0.40, -0.2, 0.0]"
export VISION_ROI_MAX="[-0.26, 0.40, 0.32]"
export MAN_EXTRA="-p vision_grasp_region_max:=[-0.15,0.40,0.30]"

# ── d(reorient) 내려앉음 막기 (2026-09-29, freeze-20260929-shaded 의 핵심) ──
#   서가 A 아래 판으로 갈 때 pre_ins 의 IK 가 먼 가지로 떨어져 손끝이 바닥선
#   아래 152 mm 까지 내려가고 물러날 때 403 이 났다. 이 문턱을 넘으면
#   ① 앞 권이 도착했던 관절값을 씨앗으로 끝 자세를 다시 풀고 ② 안 되면 되돌림
#   ③ 안 되면 d 를 나눈다. **0 이면 꺼짐 — 끄면 네 권을 못 끝낸다.**
export SIM_D_SAG_MAX_M=0.03

# ── 서가 prim (레벨 안 경로 — Collect 해도 안 바뀐다) ────────────────
export SIM_SHELF_PRIM=/World/bookshelves_main/shelf_brown__book_shelf_11
export SIM_SHELF_PRIMS="shelf_01=/World/bookshelves_main/shelf_brown__book_shelf_11;shelf_02=/World/bookshelves_main/shelf_brown__book_shelf_01"

# ── 책등 정렬은 이웃 기준 ───────────────────────────────────────────
export SIM_SPINE_FROM_NEIGHBOURS=1

# ── 네 권, 서가별로 묶어서 (B 5xx·6xx 먼저 → A 0xx) ──────────────────
export SIM_BOOK_IDS="['book_003','book_004','book_001','book_002']"
export SIM_RFID_TAGS="['rfid_003','rfid_004','rfid_001','rfid_002']"
export SIM_CLASS_CODES="['512.3','611.0','005.7','006.3']"

# ── 트레이 이송 좌표 (반납기 → 데크) ─────────────────────────────────
export SIM_TRAY_FROM="5.817607391996635,-5.659500598907469,0.333765"
export SIM_TRAY_TO="4.993110179901123,-5.659414291381836,0.333765"

# ── ROS: 팀 도메인(130)과 섞이지 않게 129 + 로컬 FastDDS 프로파일 ────
#    교육장의 팀 화이트리스트 프로파일을 쓰면 같은 PC 안 노드끼리 서로를 못 찾는다
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/b1_arm/config/fastdds_local.xml"

# ── 녹화는 끈다 (뷰포트 PNG 녹화가 시뮬을 늦춰 place_book 이 시간 초과 났다) ──
export REC_EVERY=0

# ── 교육장 주의 ─────────────────────────────────────────────────────
#   DISPLAY=:1 이어야 창이 보인다 (:0 이면 안 보인 채 돈다)
#   남의 프로세스는 끄지 않는다 — PID 로만
echo "[demo_env] SIM_LEVEL=$SIM_LEVEL"
echo "[demo_env] ROS_DOMAIN_ID=$ROS_DOMAIN_ID · REC_EVERY=$REC_EVERY"
