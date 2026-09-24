"""한 판이 안 풀리면 **다른 방법으로 다시 해 본다** — 반쪽 계획을 받아들이지 않는다.

2026-09-24 실측 (SW1):

    [스윕] 판 1.042 … 정지점 **5/5** 여유 0.210
    [스윕] 판 0.498 … 정지점 **3/5** 여유 0.100
            — x=+0.17 로 가다 경유점 8/9 IK 실패 (한계 여유 **0.059 rad** = 3.4°)
    → 404 scan > sweep_0.498_2: 제한 시간 초과

**반쪽만 풀린 계획을 그대로 실행했다.** 3개 정지점까지 가고 거기서 멈춘다.
그리고 두 판을 합쳐 정지점 8개(10개가 아니라)로 스캔이 시작되고, 못 간 구간에서
시간 초과가 난다.

왜 반쪽이 되나: 계획은 **앞 판의 끝 자세를 시드로** 이어받는다. 그 자세가 어느
IK 가지에 있느냐에 따라 다음 판이 풀리기도 하고 안 풀리기도 한다. 위 판이
여유 0.210 짜리 가지로 끝나면, 거기서 아래 판으로 내려가는 길이 막힌다.

그래서 **한 번 해 보고 포기하지 않는다.** 방향을 뒤집어 보고, 시드를 바꿔 본다.
`points` 를 줄이거나 문턱을 낮추지는 **않는다** — 그건 덜 보고 통과시키는 것이다.
"""
from __future__ import annotations

from typing import Callable, List, Optional, Sequence, Tuple


def attempts(chained_q, home_q, x_from: float, x_to: float
             ) -> List[Tuple[str, object, float, float]]:
    """해 볼 순서 — `(이름, 시드 자세, x_from, x_to)`.

    **이어가기를 먼저 둔다.** 그게 움직임이 가장 적고, 되면 그걸로 끝이다.
    그 다음이 되돌아오기(뱀 모양) — 끝난 자리에서 바로 시작하므로 이동이 없다.
    마지막이 시드 바꾸기: 앞 판의 가지를 버리고 홈 자세에서 다시 푼다.
    """
    out = [("이어가기", chained_q, x_from, x_to),
           ("되돌아오기", chained_q, x_to, x_from)]
    if home_q is not None:
        out += [("홈에서 다시", home_q, x_from, x_to),
                ("홈에서 되돌아", home_q, x_to, x_from)]
    return out


def plan_full(plan_fn: Callable[[object, float, float], object],
              cands: Sequence[Tuple[str, object, float, float]],
              points: int) -> Tuple[Optional[object], str, List[str]]:
    """`(고른 계획, 이름, 해 본 것들)`. **전부 반쪽이면 계획을 안 고른다.**

    `plan_fn(seed, x_from, x_to)` 는 `hold_idx` 와 `min_margin` 을 가진 것을 준다.

    전부 실패하면 `(None, "", 기록)` 이다 — **가장 나은 반쪽을 돌려주지 않는다.**
    반쪽을 실행하면 못 간 구간에서 시간 초과로 죽고, 그 사이 스캔은 서가의 일부를
    못 본다. 못 본 것을 못 봤다고 말하는 편이 낫다.
    """
    tried: List[str] = []
    for name, seed, a, b in cands:
        p = plan_fn(seed, a, b)
        n = len(getattr(p, "hold_idx", ()) or ())
        tried.append(f"{name} {n}/{points}"
                     + (f" 여유 {p.min_margin:.3f}" if hasattr(p, "min_margin") else ""))
        if n >= points:
            return p, name, tried
    return None, "", tried


def shrink_candidates(x_from: float, x_to: float, fracs=(1.0, 0.8, 0.6, 0.45)
                      ) -> List[Tuple[float, float]]:
    """훑을 x 범위 후보 — **닿는 데까지 줄여 본다.** 넓은 것부터.

    양쪽 끝을 각각 줄인다. 어느 쪽이 안 닿는지 모르기 때문이다 — 2026-09-24 SW2 는
    **오른쪽**이 안 닿았다(세 전략이 x=+0.175 · +0.35 에서 모두 실패).
    """
    a, b = float(x_from), float(x_to)
    span = b - a
    out: List[Tuple[float, float]] = []
    for f in fracs:
        for lo, hi in ((a, a + span * f), (b - span * f, b)):
            pair = (round(lo, 4), round(hi, 4))
            if pair not in out:
                out.append(pair)
    return out


def plan_reachable(plan_fn, seed, x_from: float, x_to: float, points: int,
                   min_span: float = 0.20):
    """닿는 만큼만 훑는 계획 — `(계획, (x_lo, x_hi), 기록)`.

    **판을 통째로 건너뛰는 것보다 낫다.** SW2 에서 아래 판(우리가 책을 꽂는 그 판)을
    한 정지점도 안 봤는데, 그 판의 **왼쪽 절반은 닿는다**. 안 닿는 것은 오른쪽뿐이다.

    못 본 구간을 **말한다.** 스캔이 서가의 일부를 못 본 채 "완료" 되면, 비전은 그
    자리의 빈칸을 영영 못 찾는다. 못 본 것을 못 봤다고 해야 한다.

    `min_span` 보다 좁아지면 포기한다 — 그만큼만 보고 "훑었다" 고 하면 거짓말이다.
    """
    tried: List[str] = []
    for lo, hi in shrink_candidates(x_from, x_to):
        if abs(hi - lo) < min_span:
            continue
        p = plan_fn(seed, lo, hi)
        n = len(getattr(p, "hold_idx", ()) or ())
        tried.append(f"[{lo:+.2f},{hi:+.2f}] {n}/{points}")
        if n >= points:
            return p, (lo, hi), tried
    return None, None, tried
