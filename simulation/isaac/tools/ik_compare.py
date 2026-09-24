#!/usr/bin/env python3
"""IK 덤프(`SIM_IK_DUMP`)를 읽어 **Isaac 없이** 두 풀이기를 대조하고, 못 푸는 목표의 원인을 가른다.

    python3 ik_compare.py <덤프.jsonl> [--pos-tol 0.001] [--rot-tol 0.01] [--json]

한 줄 = 한 IK 문제: 구간(phase) · 목표(팔 기준 손끝 자세 p_arm, R_arm, tool) · 씨앗 · Lula 해.
웹 클로드 v42 회신 §2·§3 의 실험 셋을 그대로 한다:

  모델 검증   Lula 해 q 를 ak FK(도구 프레임)에 넣어 목표와 얼마나 어긋나는가 — 목표마다
  커버리지    같은 목표를 ak 로 씨앗에서 풀어 성공/실패/오차
  판별 실험   ak 가 씨앗에서 못 푼 목표에 **Lula 해를 씨앗으로** 넣는다
              → 그 자리에서 수렴(오차 ≈ 0)이면 해는 ak 모델 안에 있다 = 순수 씨앗/유역 문제
              → 벗어나거나 오차가 남으면 모델·허용치 문제 = 씨앗을 고쳐도 안 된다

결과는 구간별로 센다. Lula 가 못 푼 줄(lula_ok false)은 커버리지에서 '둘 다 못 품' 으로만 센다.
"""
import argparse
import json
import os
import sys
from collections import OrderedDict

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

import arm_kinematics as ak          # noqa: E402


def load(path):
    out = []
    with open(path) as f:
        for i, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise SystemExit(f"{path}:{i} JSON 이 아니다: {exc}")
    return out


def problem(rec):
    """덤프 한 줄 → (p, R, tool, seed, lula_q)."""
    p = np.asarray(rec["p_arm"], float)
    R = np.asarray(rec["R_arm"], float).reshape(3, 3)
    tool = None
    if rec.get("tool"):
        tool = (np.asarray(rec["tool"]["T"], float), np.asarray(rec["tool"]["R"], float).reshape(3, 3))
    seed = np.asarray(rec["seed"], float)
    lq = None if rec.get("lula_q") is None else np.asarray(rec["lula_q"], float)
    return p, R, tool, seed, lq


def tip_error_mm(q, p, R, tool):
    pq, Rq = ak.fk_tool(q, tool)
    return float(np.linalg.norm(pq - p)) * 1000.0, float(np.linalg.norm(ak._rot_error(Rq, R)))


#: Lula 호출의 허용치 (book_scene._solve_ik 그대로). Lula 해는 이 안이면 "정답" 이다 —
#: 그러니 Lula 해를 ak FK 로 되돌린 오차가 이 안이면 모델 차이가 아니라 **Lula 의 허용치**다.
LULA_POS_TOL, LULA_ROT_TOL = 0.004, 0.05


def limit_violation(q):
    """ak 한계표 밖인 관절 → `[(관절번호, 값, 한계)]`. Lula 한계가 더 넓으면 여기서 드러난다."""
    q = np.asarray(q, float)
    out = []
    for i in range(7):
        if q[i] < ak.Q_MIN[i] - 1e-6:
            out.append((i + 1, round(float(q[i]), 4), round(float(ak.Q_MIN[i]), 4)))
        elif q[i] > ak.Q_MAX[i] + 1e-6:
            out.append((i + 1, round(float(q[i]), 4), round(float(ak.Q_MAX[i]), 4)))
    return out


def classify(p, R, tool, lq, kw):
    """ak 가 씨앗에서 못 푼 목표 하나의 원인. `(판정, 세부)`.

    2026-09-25 08:41 덤프(10,746 문제)에서 처음 판정이 "모델문제 411" 로 나왔는데 둘이 틀렸다:
    ① Lula 해를 ak FK 로 되돌린 오차 3.875 mm 는 **Lula 허용치(4 mm / 0.05 rad) 안**이다 —
       모델 차이가 아니라 Lula 가 거기서 멈춘 것이다(다른 구간은 0.002 mm 다).
    ② Lula 해가 ak 허용 안인데도 ak 가 "실패" 한 249건 — `ik()` 가 씨앗을 **한계표로 클립**하므로
       Lula 해가 ak 한계표 밖이면 클립된 씨앗은 이미 목표를 벗어난다. 모델이 아니라 **한계표**다.
    그래서 순서대로 가른다: 한계표밖 → 모델차이(Lula 허용 밖) → 씨앗문제 / 수렴실패.
    """
    viol = limit_violation(lq)
    if viol:
        return "한계표밖", {"관절": viol}
    e_mm, e_rot = tip_error_mm(lq, p, R, tool)
    if e_mm > LULA_POS_TOL * 1000.0 + 1e-6 or e_rot > LULA_ROT_TOL + 1e-6:
        return "모델차이", {"Lula해_오차mm": round(e_mm, 3), "Lula해_회전오차rad": round(e_rot, 4)}
    r2 = ak.ik(p, R, lq, tool=tool, **kw)
    moved = None if not r2.ok else float(np.max(np.abs(r2.q - lq)))
    if r2.ok and moved < 0.05:
        return "씨앗문제", {"Lula씨앗_ak_이동rad": round(moved, 4)}
    return "수렴실패", {"Lula씨앗_ak_ok": bool(r2.ok), "Lula해_오차mm": round(e_mm, 3),
                       "Lula해_회전오차rad": round(e_rot, 4)}


def compare(recs, pos_tol=0.001, rot_tol=0.01):
    """구간별 집계 + 판별 실험 결과. 돌려주는 값은 그대로 찍을 수 있는 dict."""
    kw = dict(pos_tol=pos_tol, rot_tol=rot_tol, min_margin=0.0)
    by = OrderedDict()
    verdict = OrderedDict((k, 0) for k in ("한계표밖", "모델차이", "씨앗문제", "수렴실패"))
    worst_model = 0.0
    hard = []
    points = {}
    for rec in recs:
        ph = rec.get("phase", "?")
        s = by.setdefault(ph, {"n": 0, "둘다": 0, "Lula만": 0, "ak만": 0, "둘다못": 0,
                               "모델오차mm": 0.0, "ak오차mm": 0.0})
        p, R, tool, seed, lq = problem(rec)
        s["n"] += 1
        # 1. 모델 검증 — Lula 해를 ak FK 로 (Lula 허용치 안이면 모델이 아니라 Lula 가 멈춘 자리다)
        if lq is not None:
            e_mm, _ = tip_error_mm(lq, p, R, tool)
            s["모델오차mm"] = max(s["모델오차mm"], e_mm)
            worst_model = max(worst_model, e_mm)
        # 2. 커버리지 — ak 를 씨앗에서
        r = ak.ik(p, R, seed, tool=tool, **kw)
        if r.ok:
            e_mm, _ = tip_error_mm(r.q, p, R, tool)
            s["ak오차mm"] = max(s["ak오차mm"], e_mm)
        if lq is not None and r.ok:
            s["둘다"] += 1
        elif lq is not None:
            s["Lula만"] += 1
            why, detail = classify(p, R, tool, lq, kw)      # 3. 판별
            verdict[why] += 1
            key = tuple(round(float(v), 3) for v in rec["p_arm"])
            pt = points.setdefault(key, {"n": 0, "판정": {}})
            pt["n"] += 1
            pt["판정"][why] = pt["판정"].get(why, 0) + 1
            hard.append(dict({"phase": ph, "p_arm": rec["p_arm"], "판정": why,
                              "ak_씨앗풀이_오차mm": round(r.pos_err * 1000, 2)}, **detail))
        elif r.ok:
            s["ak만"] += 1
        else:
            s["둘다못"] += 1
    return {"구간": by, "판별": verdict, "모델오차_최대mm": round(worst_model, 3), "못푼목표": hard,
            "서로다른점": len(points), "점별": points,
            "허용": {"pos_tol": pos_tol, "rot_tol": rot_tol, "Lula_pos_tol": LULA_POS_TOL, "Lula_rot_tol": LULA_ROT_TOL}}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dump")
    ap.add_argument("--pos-tol", type=float, default=0.001, help="ak 위치 허용 (m). 기본 1 mm")
    ap.add_argument("--rot-tol", type=float, default=0.01, help="ak 회전 허용 (rad)")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    recs = load(a.dump)
    out = compare(recs, a.pos_tol, a.rot_tol)
    if a.json:
        print(json.dumps(out, ensure_ascii=False, indent=1))
        return 0
    print(f"덤프 {a.dump} · 문제 {len(recs)}개 · ak 허용 {a.pos_tol * 1000:.1f} mm / {a.rot_tol} rad")
    print(f"모델 검증: Lula 해를 ak FK(도구 프레임)로 되돌린 손끝 오차 최대 **{out['모델오차_최대mm']:.3f} mm**")
    print()
    print(f"{'구간':>14}{'n':>6}{'둘다':>6}{'Lula만':>7}{'ak만':>6}{'둘다못':>7}{'모델오차mm':>11}{'ak오차mm':>10}")
    print("-" * 68)
    for ph, s in out["구간"].items():
        print(f"{ph:>14}{s['n']:>6}{s['둘다']:>6}{s['Lula만']:>7}{s['ak만']:>6}{s['둘다못']:>7}"
              f"{s['모델오차mm']:>11.3f}{s['ak오차mm']:>10.3f}")
    v = out["판별"]
    print()
    print(f"모델 검증 기준: Lula 허용치 {LULA_POS_TOL * 1000:.0f} mm / {LULA_ROT_TOL} rad 안이면 모델 차이가 아니다")
    print("판별 실험 (ak 가 씨앗에서 못 푼 것): " + " · ".join(f"{k} {n}" for k, n in v.items())
          + f"   — 서로 다른 점 {out['서로다른점']}개")
    viol = {}
    for h in out["못푼목표"]:
        for j, val, lim in h.get("관절", []):
            k = (j, "아래" if val < lim else "위")
            viol[k] = viol.get(k, 0) + 1
    if viol:
        print("  한계표밖 — 어느 관절이 ak 한계표를 넘는가: "
              + " · ".join(f"관절 {j} {side}로 {n}건" for (j, side), n in sorted(viol.items())))
    for key, pt in sorted(out["점별"].items(), key=lambda kv: -kv[1]["n"])[:12]:
        print(f"  {list(key)}  {pt['n']}번  " + " · ".join(f"{k} {n}" for k, n in pt["판정"].items()))
    if out["서로다른점"] > 12:
        print(f"  … 점 {out['서로다른점'] - 12}개 더 (--json 으로 전부)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
