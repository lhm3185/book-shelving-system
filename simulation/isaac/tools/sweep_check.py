#!/usr/bin/env python3
"""경로 스침 검사 — 계획 덤프(`SIM_PLAN_DUMP`)를 FK 로 훑어 **쥔 책이 이웃 책과 얼마나 가까이 지나는지**를 mm 로.

    python3 sweep_check.py <plan.json> [--json]

서가 책에는 콜리전이 없다(설계). 팔이 관통해도 책이 안 움직이니 "이웃이 움직였나" 검사는 항상 통과하는
헛검사다 — 스침은 자세 변화가 아니라 **기하**로 잡는다(웹 클로드 v44 회신 §5). 계획 경유점마다 손끝 FK
(팔 기준, `arm_kinematics.fk_tool`)를 월드로 바꾸고, 쥔 책의 상자(손끝 아래로 매달린 책)를 그 자세로 놓아
이웃 책 AABB 와의 이격을 센다. **보고 전용** — 문턱을 미리 만들지 않고 최소 이격을 구간별로 찍는다.

근사: 쥔 책 상자는 손끝 프레임에서 x·y 반폭을 max(두께, 폭)/2 로 잡는다(물림축이 GRIP_ROT90 에 따라
바뀌므로 보수적으로 둘 다 덮는다), z 는 손끝 위 tip_down 부터 아래로 책 높이만큼. 상자를 회전시켜 월드
AABB 로 감싸므로 실제보다 **넓게** 본다 — 여기서 0 이면 진짜 닿았을 수 있고, 여유가 있으면 확실히 안 닿는다.
"""
import argparse
import json
import os
import sys
from collections import OrderedDict

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

import arm_kinematics as ak          # noqa: E402


def held_book_box_local(dims, tip_down):
    """쥔 책의 상자 — 손끝 프레임 `(min xyz, max xyz)`. dims = (두께 T, 높이 Lb, 폭 W).

    Franka 손가락은 손 **y** 로 벌어지고 닫힌다 → 책 두께(T)는 손끝 y, 폭(W)은 손끝 x 방향이다.
    처음엔 둘 다 max(T, W)/2 로 잡았는데 그러면 밀어 넣는 구간(push)에서 이웃과 -25.9 mm 로 "겹쳐"
    보였다 — 같은 판 `[겹침]` 실측은 여유 14.7 mm. 두께 방향을 제대로 두면 그 허위가 사라진다
    (2026-09-25 데스크탑 대조). 축 가정이 틀리면 이격이 작게 나오는 쪽(보수)이 아니라 크게 나올 수
    있으니, `[겹침]` 실측과 나란히 본다.
    """
    T, Lb, W = (float(v) for v in dims)
    return (np.array([-W / 2.0, -T / 2.0, -(Lb - float(tip_down))]),
            np.array([W / 2.0, T / 2.0, float(tip_down)]))


def box_corners(lo, hi):
    lo = np.asarray(lo, float); hi = np.asarray(hi, float)
    return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


def aabb_of_points(pts):
    pts = np.asarray(pts, float)
    return np.concatenate([pts.min(axis=0), pts.max(axis=0)])


def aabb_gap(a, b):
    """두 AABB(x0 y0 z0 x1 y1 z1) 사이 **축별 최대 이격**(m). 겹치면 음수(가장 얕은 관통 깊이의 음수)."""
    a = np.asarray(a, float); b = np.asarray(b, float)
    gaps = np.maximum(b[:3] - a[3:], a[:3] - b[3:])       # 축별 이격 (겹치면 음수)
    if np.all(gaps < 0):
        return float(gaps.max())                            # 전 축 겹침 = 관통, 가장 얕은 축의 깊이
    return float(gaps.max())


def sweep(plan):
    """덤프 → 구간별 최소 이격. `(per_label OrderedDict, worst_overall)` — 이격 mm, 어느 이웃, 몇 번째 점."""
    l0p = np.asarray(plan["l0p"], float)
    Rl0 = np.asarray(plan["Rl0"], float).reshape(3, 3)
    tool = None
    if plan.get("tool"):
        tool = (np.asarray(plan["tool"]["T"], float), np.asarray(plan["tool"]["R"], float).reshape(3, 3))
    lo, hi = held_book_box_local(plan["book_dims"], plan["tip_down"])
    local = box_corners(lo, hi)
    neigh = [np.asarray(b, float) for b in plan.get("neighbours", [])]
    per = OrderedDict()
    worst = None
    for i, (q, lab) in enumerate(zip(plan["waypoints"], plan["labels"])):
        p, R = ak.fk_tool(q, tool)                    # 팔 기준 손끝
        pw = l0p + Rl0 @ p
        Rw = Rl0 @ R
        box = aabb_of_points((Rw @ local.T).T + pw)
        best = None
        for j, nb in enumerate(neigh):
            g = aabb_gap(box, nb)
            if best is None or g < best[0]:
                best = (g, j)
        if best is None:
            continue
        s = per.setdefault(lab, {"n": 0, "min_mm": None, "at": None, "neighbour": None})
        s["n"] += 1
        if s["min_mm"] is None or best[0] * 1000.0 < s["min_mm"]:
            s["min_mm"] = round(best[0] * 1000.0, 1); s["at"] = i; s["neighbour"] = best[1]
        if worst is None or best[0] < worst[0]:
            worst = (best[0], lab, i, best[1])
    return per, worst


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("plan")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    with open(a.plan) as f:
        plan = json.load(f)
    for k in ("l0p", "Rl0", "book_dims", "tip_down", "waypoints", "labels"):
        if k not in plan:
            raise SystemExit(f"덤프에 {k} 가 없다 — 스침 검사용 부가 정보가 든 덤프(2026-09-25 이후)여야 한다")
    per, worst = sweep(plan)
    if a.json:
        print(json.dumps({"segments": per, "worst": worst}, ensure_ascii=False, indent=1))
        return 0
    print(f"덤프 {a.plan} · 경유점 {len(plan['waypoints'])} · 이웃 {len(plan.get('neighbours', []))}권 · "
          f"쥔 책 {[round(v * 1000) for v in plan['book_dims']]} mm  (보고 전용 — 문턱 없음)")
    print(f"{'구간':>14}{'점':>6}{'최소 이격 mm':>14}{'어느 점':>9}{'이웃#':>7}")
    for lab, s in per.items():
        print(f"{lab:>14}{s['n']:>6}{s['min_mm']:>14}{s['at']:>9}{s['neighbour']:>7}")
    if worst:
        print(f"\n경로 전체 최소 이격 **{worst[0] * 1000:.1f} mm** ({worst[1]} {worst[2]}번째 점, 이웃 #{worst[3]})"
              + ("  ← 음수 = 상자가 겹친다 (보수적 상자라 실제 접촉일 수도, 아닐 수도)" if worst[0] < 0 else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
