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


def compare(recs, pos_tol=0.001, rot_tol=0.01):
    """구간별 집계 + 판별 실험 결과. 돌려주는 값은 그대로 찍을 수 있는 dict."""
    kw = dict(pos_tol=pos_tol, rot_tol=rot_tol, min_margin=0.0)
    by = OrderedDict()
    verdict = {"씨앗문제": 0, "모델문제": 0}
    worst_model = 0.0
    hard = []
    for rec in recs:
        ph = rec.get("phase", "?")
        s = by.setdefault(ph, {"n": 0, "둘다": 0, "Lula만": 0, "ak만": 0, "둘다못": 0,
                               "모델오차mm": 0.0, "ak오차mm": 0.0})
        p, R, tool, seed, lq = problem(rec)
        s["n"] += 1
        # 1. 모델 검증 — Lula 해를 ak FK 로
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
            # 3. 판별 — Lula 해를 씨앗으로
            r2 = ak.ik(p, R, lq, tool=tool, **kw)
            e_mm, e_rot = tip_error_mm(lq, p, R, tool)
            moved = None if not r2.ok else float(np.max(np.abs(r2.q - lq)))
            if r2.ok and moved is not None and moved < 0.05:
                verdict["씨앗문제"] += 1
                why = "씨앗문제"
            else:
                verdict["모델문제"] += 1
                why = "모델문제"
            hard.append({"phase": ph, "p_arm": rec["p_arm"], "판정": why,
                         "Lula해_오차mm": round(e_mm, 3), "Lula해_회전오차rad": round(e_rot, 4),
                         "Lula씨앗_ak_ok": bool(r2.ok),
                         "Lula씨앗_ak_이동rad": None if moved is None else round(moved, 4),
                         "ak_씨앗풀이_오차mm": round(r.pos_err * 1000, 2)})
        elif r.ok:
            s["ak만"] += 1
        else:
            s["둘다못"] += 1
    return {"구간": by, "판별": verdict, "모델오차_최대mm": round(worst_model, 3), "못푼목표": hard,
            "허용": {"pos_tol": pos_tol, "rot_tol": rot_tol}}


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
    print(f"판별 실험 (Lula 해를 ak 씨앗으로): 씨앗문제 {v['씨앗문제']} · 모델문제 {v['모델문제']}")
    for h in out["못푼목표"][:20]:
        mv = h["Lula씨앗_ak_이동rad"]
        print(f"  {h['phase']:>12} p_arm {h['p_arm']}  → **{h['판정']}**  "
              f"Lula해 오차 {h['Lula해_오차mm']:.3f} mm / {h['Lula해_회전오차rad']:.4f} rad · "
              f"Lula씨앗 ak {'ok' if h['Lula씨앗_ak_ok'] else '실패'}"
              + ("" if mv is None else f" 이동 {mv:.4f} rad"))
    if len(out["못푼목표"]) > 20:
        print(f"  … {len(out['못푼목표']) - 20}개 더 (--json 으로 전부)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
