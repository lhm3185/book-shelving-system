"""계획 전체 감사 — 지금 계획의 모든 경유점을 '팔꿈치↑ 가지'로 옮길 수 있는지 본다.

입력: plan_job 이 만든 경유점 관절각을 JSON 으로 덤프한 파일
    {"home": [6], "waypoints": [[6], ...], "labels": ["approach", ...], "linear": [false, ...]}
    labels · linear 는 선택. linear=true 는 movel(직교 보간) 구간의 점이다.

    python3 m0609_plan_audit.py plan_dump.json
    python3 m0609_plan_audit.py plan_dump.json --home-up -0.1513 -0.2792 -1.2832 0.0 -1.5795 -3.2930

하는 일: 각 경유점의 손 자세(link_6)를 그대로 두고, 팔꿈치↑ 해 중
**직전 점과 가장 가까운 것**을 골라 이어 붙인다. 그리고
  · 팔꿈치↑ 해가 아예 없는 점          → ✗ (여기서 경로 A 가 막힌다)
  · 받침판 여유 < 0.02 m 인 점         → △ (충돌 구 근사값 기준)
  · movel 구간에서 인접점 관절변화 > 0.1 rad → ✗ (직선 보간이 가지를 건넌다)
를 표시한다. 서가·트레이 벽·그리퍼는 모델에 없다 — 팔 링크와 받침판만 본다.
"""
import argparse
import json
import numpy as np
from m0609_kin import pose6, all_solutions, elbow_branch, deck_clearance

HOME_UP = [-0.1513, -0.2792, -1.2832, 0.0, -1.5795, -3.2930]   # 지금 홈과 같은 손 자세, 팔꿈치↑

ap = argparse.ArgumentParser()
ap.add_argument("dump")
ap.add_argument("--home-up", nargs=6, type=float, default=HOME_UP)
a = ap.parse_args()
d = json.load(open(a.dump, encoding="utf-8"))
wps = [np.asarray(w, float) for w in d["waypoints"]]
labels = d.get("labels") or [f"wp{i}" for i in range(len(wps))]
linear = d.get("linear") or [False] * len(wps)

prev = np.asarray(a.home_up, float)
hp, _ = pose6(prev)
print(f"시작 = 팔꿈치↑ 홈  link_6 {np.round(hp, 4)}  (지금 홈과 같아야 한다: [-0.4748 0.0788 0.4117])\n")
bad = 0
for i, (q_now, lab, lin) in enumerate(zip(wps, labels, linear)):
    tp, tR = pose6(q_now)
    br_now, _ = elbow_branch(q_now)
    ups = [q for q in all_solutions(tp, tR) if elbow_branch(q)[0] == "up"]
    if not ups:
        print(f"✗ {i:3d} {lab:18s} 팔꿈치↑ 해 없음 (지금 계획은 {br_now})"); bad += 1; continue
    q_up = min(ups, key=lambda q: np.max(np.abs(q - prev)))
    step = float(np.max(np.abs(q_up - prev)))
    cl, who = deck_clearance(q_up)
    mark = "✓"
    if cl < 0.02:
        mark = "△"
    if lin and step > 0.1:
        mark = "✗"; bad += 1
    print(f"{mark} {i:3d} {lab:18s} 지금 {br_now:4s} → 팔꿈치↑ 여유 {cl:+.3f} ({who})  "
          f"직전 대비 {step:.3f} rad{'  [movel]' if lin else ''}")
    prev = q_up
print(f"\n{'경로 A 로 전 구간 이어짐' if bad == 0 else f'막히는 점 {bad}개 — 경로 A 재검토'}")
