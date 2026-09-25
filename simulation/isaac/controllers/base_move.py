"""자리 맞추기 이동량의 상한 — **이상치를 자른다.**

`rotate_base` 는 회전 뒤에 서가까지 거리(`standoff`)와 옆이동을 맞추려고 차체를
옮긴다. 그런데 그 이동량에 상한이 없었다. 2026-09-24 VD2 에서 **키오스크에 선 채로
서가 자리 맞추기 명령이 실행되어** 차체를 2.676 m 끌고 가려 했고, 그 판은 418초를
먹고 끝났다.

**이 상한은 문턱을 올려 통과시키는 것이 아니다 — 그 반대다.** 43회 실측 분포에서
정상 최대와 6배 떨어진 값 하나를 거절한다.

    0.266 m   24회   스캔 전 중심 맞추기 (서가가 고정이라 항상 같다)
    0.267      3
    0.273      5  ┐
    0.274      4  │ 꽂기 전, 실측 빈칸 맞춤이 들어간 판
    0.288~0.303 5 │
    0.444      1  │
    0.447      1  ┘ 맞춤 없던 판의 최대  ← **정상 최대**
    ─────────────────────────────────
    2.676      1   VD2 하나                ← 정상 최대의 6.0배

기본 상한 0.6 m 는 정상 최대(0.447)에 34% 여유를 두고, VD2(2.676)보다 4.5배 아래다.
**두 무리 사이가 이렇게 비어 있어서** 고를 수 있는 값이다. 분포가 이어져 있었으면
상한으로 가르지 않고 원인을 먼저 봤을 것이다.

주의: **이것은 증상을 자르는 것이지 원인이 아니다.** 원인은 "사이클이 포기한 뒤에도
철 지난 명령이 실행된다" 쪽에 있고, 그건 따로 본다. 상한은 즉효약이다.
"""
from __future__ import annotations

import os

#: 실측 정상 최대 (m). 맞춤 없던 판의 꽂기 전 이동
OBSERVED_MAX = 0.447
#: 관측된 이상치 (m). VD2 가 키오스크에서 서가 자리를 맞추려 한 거리
OBSERVED_OUTLIER = 2.676


def move_limit_m() -> float:
    """상한 (m). `SIM_ROTATE_MAX_MOVE` 로 바꾼다. 0 이하면 **상한을 끈다.**"""
    try:
        return float(os.environ.get("SIM_ROTATE_MAX_MOVE", "0.6"))
    except ValueError:
        return 0.6


def within_shelf_span(x_before: float, x_after: float, span, margin: float = 0.05) -> bool:
    """옆이동 **전·후**의 팔 베이스 x(팔 기준)가 둘 다 그 서가의 x 구간 안(여유 margin)인가.

    거리 상한은 "얼마나 멀리" 만 보고 "어디서 어디로" 를 안 본다. 서가 B(2026-09-25)는 빈칸이 양 끝이라
    중심에서 95.8 mm 칸까지 0.624 m — 실측 정상 최대(0.447)에서 나온 상한 0.6 을 4 % 넘겼는데, 그 이동은
    서가 폭(1.40 m) 안에서 서가를 따라가는 이동이다. 반면 원래 막으려던 사고(VD2, 키오스크에 선 채로
    2.676 m)는 출발점부터 서가 구간 밖이다. 그래서 **서가 구간 안에서 서가를 따라가는 이동**이면 거리
    상한을 안 보고, 아니면 본다 — 숫자가 아니라 뜻으로 막는다. `span` 이 없으면 False(상한으로 돌아간다).
    """
    if not span:
        return False
    lo, hi = sorted((float(span[0]), float(span[1])))
    return (lo - margin <= float(x_before) <= hi + margin) and (lo - margin <= float(x_after) <= hi + margin)


def refuse_far_move(dist_m: float, limit_m: float | None = None, along_shelf: bool = False) -> str | None:
    """너무 멀면 **거절 사유**를, 괜찮으면 `None` 을 돌려준다.

    거절은 말없이 하지 않는다. 사유에 실측 정상 최대를 같이 실어서, 읽는 사람이
    "상한이 빡빡한 것" 과 "이번 값이 이상한 것" 을 구분할 수 있게 한다.
    `along_shelf=True`(`within_shelf_span` 으로 확인한 것)면 서가 폭 안의 이동이라 상한을 안 본다.
    """
    lim = move_limit_m() if limit_m is None else float(limit_m)
    if lim <= 0.0:
        return None                       # 상한을 껐다 (회귀 비교용)
    d = float(dist_m)
    if d <= lim or along_shelf:
        return None
    return (f"자리 맞추기 이동 {d:.3f} m 가 상한 {lim:.3f} m 를 넘는다 "
            f"(실측 정상 최대 {OBSERVED_MAX:.3f} m). "
            f"서가가 아닌 자리에서 서가 자리를 맞추려는 명령일 수 있다 — 하지 않는다")
