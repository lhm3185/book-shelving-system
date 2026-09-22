"""가지(branch) 점검기 — 받침판 막힘이 '창이 없어서'인지 '가지가 틀려서'인지 가른다.

    python3 m0609_branch_check.py                    # v21 의 approach 목표로
    python3 m0609_branch_check.py --q -0.257 -1.848 0.938 1.727 -1.768 0.681

같은 손 자세(link_6 위치+방향)를 내는 해 8개를 전부 구하고, 가지마다
  · 팔꿈치 위/아래, 윗팔 앙각
  · 받침판 최소 여유 (충돌 구 기준 — 근사값, m0609_kin.py 머리말 참조)
  · 수직 10 cm down 을 직선으로 따라갈 때 스텝당 최대 관절변화 · σmin
을 찍는다. σmin 이 경로 내내 0 에 다가가지 않으면 특이점이 아니다.

④ 음성 대조: 첫 줄에 '받침판을 뚫는다고 알려진 자세'(v21 approach 목표)를 넣어
             여유가 음수로 나오는지 먼저 본다. 안 나오면 이 도구를 믿지 말 것.
"""
import argparse
import numpy as np
from m0609_kin import pose6, all_solutions, elbow_branch, deck_clearance, track_line

KNOWN_BAD = [-0.257, -1.848, 0.938, 1.727, -1.768, 0.681]   # v21 §3 — 실제로 받침판에 막힌 목표

ap = argparse.ArgumentParser()
ap.add_argument("--q", nargs=6, type=float, default=KNOWN_BAD, help="이 관절각이 내는 손 자세로 점검")
ap.add_argument("--down", type=float, default=0.10, help="수직으로 내릴 거리 (m)")
a = ap.parse_args()

cl, who = deck_clearance(KNOWN_BAD)
print(f"④ 음성 대조 — 막혔던 자세의 받침판 여유 {cl:+.3f} m ({who})  "
      f"{'→ 음수: 도구가 막힘을 본다' if cl < 0 else '→ 양수: 도구가 막힘을 못 본다. 여기서 멈출 것'}\n")

tp, tR = pose6(a.q)
print(f"손(link_6) 위치 {np.round(tp, 3)}  팔 축에서 수평 {np.hypot(tp[0], tp[1]):.3f} m\n")
for i, q in enumerate(all_solutions(tp, tR)):
    br, elev = elbow_branch(q)
    c, w = deck_clearance(q)
    d = track_line(q, tp, tR, dz=-a.down)
    dd = "down 해 끊김" if d is None else (
        f"down 스텝당 최대 {d['max_step']:.3f} rad · σmin {d['sigma_min']:.3f} · 최소여유 {d['min_clearance']:+.3f}")
    print(f"[{i}] 팔꿈치{'↑' if br == 'up' else '↓'} 윗팔 {elev:+5.1f}°  j={np.round(q, 3).tolist()}\n"
          f"     여유 {c:+.3f} ({w})  |  {dd}")
