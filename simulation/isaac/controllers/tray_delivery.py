"""트레이 이송 좌표를 정한다 — **레벨이 정답이다**.

왜 따로 뺐나: 이 판단에는 Isaac 이 필요 없다. 순수 산수다. `book_scene.py` 안에
있으면 GPU 없이는 한 줄도 시험할 수 없어서, 틀린 채로 밤을 넘겼다.

무엇이 틀렸었나: "레벨에서 잰 값" 이라 주석을 달아 놓고 **그 값을 상수로 박았다.**
레벨이 바뀌자 그대로 썩었다 — 트레이 원점이 z 0.37 → 0.333765 로 내려갔는데 코드는
0.37 을 출발점으로 알고 있었다. 도착점은 그 차이만큼 끌려 내려가 데크 아래를
겨눴다. 2026-09-23 밤의 410·409 가 여기서 시작됐다.

규칙은 하나다. **관계를 계산하고 상수를 박지 않는다.**
"""
from __future__ import annotations

import os

import numpy as np

#: 레벨 트레이를 못 찾았을 때만 쓰는 대체값 (2026-09-24 레벨 실측).
#: **이 값을 기본값으로 쓰지 않는다** — 레벨에서 읽는 것이 기본이다.
TRAY_FROM_FALLBACK = [5.817607391996635, -5.659500598907469, 0.333765]

#: 도착 자리의 x·y. **z 는 적지 않는다 — 출발 z 를 그대로 쓴다** (2026-09-23 도윤님 지시).
#: 데크 위 6.2 cm 에서 놓이지만 콜리전이 딱 맞지 않아 책이 튀지 않고 안착한다.
TRAY_TO_XY = [4.993110179901123, -5.659414291381836]


def _xyz(name: str, default, env=None):
    """환경변수 "x,y,z" → 배열. 비어 있으면 기본값."""
    e = os.environ if env is None else env
    v = str(e.get(name, "")).strip()
    if not v:
        return np.array(default, float)
    return np.array([float(t) for t in v.replace(" ", "").split(",")], float)


def deliver_from_to(level_p, env=None):
    """이송 출발·도착 좌표를 정한다 → (from, to, 출처).

    출발은 레벨의 트레이 원점 그대로. 도착은 x·y 만 바꾸고 **z 는 출발과 같게** 둔다.
    환경변수(`SIM_TRAY_FROM`·`SIM_TRAY_TO`)를 주면 그것이 이긴다 — A/B·회귀 비교용이다.
    """
    e = os.environ if env is None else env
    if str(e.get("SIM_TRAY_FROM", "")).strip():
        src = "SIM_TRAY_FROM"
        frm = _xyz("SIM_TRAY_FROM", TRAY_FROM_FALLBACK, env)
    elif level_p is None:
        src = "대체값(레벨 트레이 못 찾음)"
        frm = np.array(TRAY_FROM_FALLBACK, float)
    else:
        src = "레벨"
        frm = np.asarray(level_p, float).copy()
    to = _xyz("SIM_TRAY_TO", list(TRAY_TO_XY) + [frm[2]], env)
    return np.asarray(frm, float), np.asarray(to, float), src


def drift_mm(frm) -> float:
    """기록해 둔 레벨 값과 얼마나 다른가 (mm). 레벨이 바뀌었는지 알리는 용도."""
    return abs(float(frm[2]) - TRAY_FROM_FALLBACK[2]) * 1000.0
