# 비전이 **본 것을 잰 그대로** 내는 파지 관측.
#
# `TargetSlot.pose` 와 달리 이것은 **AABB 중심이 아니다.** 보이는 윗면의 중심이다.
# 이름을 나눈 이유: 같은 필드 이름에 서로 다른 점을 넣어 8cm·0.67m 가 어긋난 적이 있다
# (2026-09-17 사고 #3·#5). `pose` 는 좌표 계약 2번(AABB 중심) 전용으로 남긴다.
#
# 계약: docs/doyoon-kim/manipulation/PICKPLACE_CONTRACT_V2.md §3

std_msgs/Header header
# frame_id 는 반드시 arm_base_link. stamp 는 **관측한 영상의 시각**
# (요청 시각이 아니다 — 받는 쪽이 나이를 보고 오래되면 411 로 거절한다)

geometry_msgs/Point top_center
# 보이는 **윗면의 중심**. 책등 중앙의 한 점도, AABB 중심도 아니다.
# 트레이에 세운 책이면 바닥에서 책 폭(W)만큼 위에 있어야 한다 — 받는 쪽이 대조한다

float32 spine_yaw       # 책등이 향하는 방향 (arm_base_link z 축 기준, rad)
float32 thickness       # 윗면에서 잰 두께 (m)
float32 width           # 윗면에서 잰 폭 (m)
float32 confidence      # 0~1
