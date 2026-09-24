#!/usr/bin/env python3
"""로그에 찍힌 **두 자세**만으로 "가지 넘김인가, 진짜 못 가는가" 를 GPU 없이 가른다.

왜 필요한가
-----------
2026-09-24 밤, 스윙 운반의 `c(reach)` 가 위 판에서만 401 로 죽는다. 32배까지 잘게
나눠도 한 걸음이 0.126~0.168 rad 남는다. 이 숫자만으로는 두 가지를 못 가른다:

  ① **가지 넘김** — 손끝은 거의 제자리인데 IK 가 팔 모양이 다른 해로 튀었다.
     잘게 나눠도 안 줄어든다(불연속이라). 관절공간으로 이으면 지나가긴 한다.
  ② **도달 한계** — 손끝이 정말 갈 수 없는 데로 간다. 관절공간으로 이어도
     끝점이 안 풀리거나, 풀려도 가는 길에 팔이 엉뚱한 데를 지난다.

가르는 법은 간단하다. `[IK] 튐 지점` 이 찍어 주는 **앞·뒤 자세를 각각 FK** 해서
손끝을 비교한다. 손끝이 거의 같은데 관절이 멀면 ①이다. 손끝도 멀면 ②다.

그리고 ①이라 해도 **관절공간으로 이어도 되는지는 따로 물어야 한다.** 관절공간
보간은 양 끝만 맞고 사이는 아무 데나 지난다 — 2026-09-22 저녁 영상에서 손끝이
1 m 넘게 솟은 게 그것이고, 스윙 분해는 그걸 없애려고 만든 것이다. 그래서 여기서
보간 경로를 전부 FK 해서 **직선에서 얼마나 벗어나는지**를 같이 낸다.

    python3 swing_check.py --before "0.1,-0.5,..." --after "..."
    python3 swing_check.py --before "..." --after "..." --json

한계
----
기구학은 `arm_kinematics`(Franka MDH)다. 시뮬은 Lula 를 쓴다. 두 모형이 같은지는
`book_scene.check_ak_model()` 로 재는 중이고 아직 결론이 없다 — 그러니 여기 숫자는
**방향을 가르는 데** 쓰고, 최종 판정은 판에서 받는다. 받침판·서가 충돌은 안 본다.
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

import arm_kinematics as ak          # noqa: E402

#: 손끝이 이만큼 안 움직였으면 "제자리" 로 본다 (m). 한 걸음 직교 간격 0.005 의 두 배
SAME_TIP_M = 0.010
#: 관절이 이만큼 이상 벌어졌으면 "먼 해" 로 본다 (rad). plan_path 의 MAX_STEP 이 0.12
FAR_Q_RAD = 0.12


def parse_q(text):
    """`0.1,-0.5,...` 또는 `[0.1, -0.5, ...]` 를 7축 배열로."""
    t = text.strip().strip("[]")
    vals = [float(v) for v in t.replace(",", " ").split()]
    if len(vals) < 7:
        raise SystemExit(f"관절값이 {len(vals)}개다 — 7개가 필요하다: {text}")
    return np.asarray(vals[:7], float)


def rot_angle(Ra, Rb):
    """두 회전 사이 각 (rad)."""
    c = (float(np.trace(Ra.T @ Rb)) - 1.0) / 2.0
    return float(np.arccos(max(-1.0, min(1.0, c))))


def envelope(qs, p_from, p_to):
    """관절 경로가 **직선 p_from→p_to 에서 얼마나 벗어나는가** (최대 이탈 m, 최대 상승 m)."""
    a = np.asarray(p_from, float)
    b = np.asarray(p_to, float)
    ab = b - a
    L2 = float(ab @ ab)
    dev = 0.0
    rise = 0.0
    for q in qs:
        p, _R = ak.fk(q)
        t = 0.0 if L2 < 1e-12 else float(np.clip((p - a) @ ab / L2, 0.0, 1.0))
        dev = max(dev, float(np.linalg.norm(p - (a + ab * t))))
        rise = max(rise, float(p[2] - max(a[2], b[2])))
    return dev, rise


def compare(q_before, q_after, step=0.084):
    """앞·뒤 두 자세를 재서 판정과 숫자를 돌려준다. step 기본 = MAX_STEP(0.12)×0.7."""
    p0, R0 = ak.fk(q_before)
    p1, R1 = ak.fk(q_after)
    d_tip = float(np.linalg.norm(p1 - p0))
    d_rot = rot_angle(R0, R1)
    dq = np.abs(np.asarray(q_after, float) - np.asarray(q_before, float))
    path = [np.asarray(q_before, float)] + ak.move_j(q_before, q_after, step=step)
    dev, rise = envelope(path, p0, p1)
    if d_tip <= SAME_TIP_M and float(dq.max()) >= FAR_Q_RAD:
        verdict = "가지 넘김"
        why = (f"손끝이 {d_tip * 1000:.1f} mm 밖에 안 움직였는데 관절은 "
               f"{float(dq.max()):.3f} rad 튀었다 — 같은 손 자세의 **다른 팔 모양**이다")
    elif d_tip > SAME_TIP_M:
        verdict = "진짜 이동"
        why = (f"손끝이 {d_tip * 1000:.1f} mm 움직였다 — 가지가 아니라 목표가 이동한 것이다. "
               f"걸음이 큰 건 그 구간이 특이점에 가깝기 때문일 수 있다")
    else:
        verdict = "판정 불가"
        why = f"손끝 {d_tip * 1000:.1f} mm · 관절 최대 {float(dq.max()):.3f} rad — 둘 다 작다"
    return {
        "판정": verdict,
        "까닭": why,
        "손끝_앞": np.round(p0, 4).tolist(),
        "손끝_뒤": np.round(p1, 4).tolist(),
        "손끝_이동_mm": round(d_tip * 1000, 2),
        "손자세_차이_deg": round(np.degrees(d_rot), 2),
        "관절_차이_rad": np.round(dq, 4).tolist(),
        "관절_최대_rad": round(float(dq.max()), 4),
        "튄_관절": int(np.argmax(dq)) + 1,
        "한계여유_앞": round(ak.limit_margin(q_before), 4),
        "한계여유_뒤": round(ak.limit_margin(q_after), 4),
        "관절보간_점수": len(path),
        "관절보간_직선이탈_m": round(dev, 4),
        "관절보간_상승_m": round(rise, 4),
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--before", required=True, help="`[IK] 튐 지점` 의 앞 자세 (관절 7개)")
    ap.add_argument("--after", required=True, help="같은 줄의 뒤 자세 (관절 7개)")
    ap.add_argument("--step", type=float, default=0.084,
                    help="관절 보간 간격 rad (기본 0.084 = MAX_STEP 0.12 × 0.7)")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)

    r = compare(parse_q(a.before), parse_q(a.after), step=a.step)
    if a.json:
        print(json.dumps(r, ensure_ascii=False, indent=2))
        return 0

    print(f"판정: **{r['판정']}**")
    print(f"  {r['까닭']}")
    print()
    print(f"  손끝  앞 {r['손끝_앞']}  →  뒤 {r['손끝_뒤']}")
    print(f"        이동 {r['손끝_이동_mm']} mm · 손 자세 차이 {r['손자세_차이_deg']}°")
    print(f"  관절  최대 {r['관절_최대_rad']} rad (관절 {r['튄_관절']})")
    print(f"        {r['관절_차이_rad']}")
    print(f"  한계 여유  앞 {r['한계여유_앞']} → 뒤 {r['한계여유_뒤']}")
    print()
    print(f"  이 둘을 **관절공간으로 이으면** ({r['관절보간_점수']}점):")
    print(f"        직선에서 최대 {r['관절보간_직선이탈_m'] * 1000:.0f} mm 벗어난다")
    print(f"        두 끝보다 최대 {r['관절보간_상승_m'] * 1000:.0f} mm 솟는다")
    print()
    if r["판정"] == "가지 넘김" and r["관절보간_직선이탈_m"] <= 0.05:
        print("  → 관절공간 되돌림이 **안전해 보인다**. 판에서 확인할 것.")
    elif r["판정"] == "가지 넘김":
        print("  → 가지 넘김은 맞지만 **관절공간으로 이으면 크게 휘두른다**. 그대로 쓰지 말 것.")
    else:
        print("  → 관절공간 되돌림으로 풀릴 문제가 아니다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
