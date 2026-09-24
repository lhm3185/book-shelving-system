"""오프라인 IK 대조 — 덤프 네 필드로 모델 검증·커버리지·판별 실험을 Isaac 없이.

웹 클로드 v42 회신 §2·§3. 판별의 핵심: ak 가 씨앗에서 못 푼 목표에 Lula 해를 씨앗으로 넣어
그 자리에서 수렴하면 "씨앗 문제", 아니면 "모델 문제". 여기서 두 경우를 만들어 가른다.
"""

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

import arm_kinematics as ak                     # noqa: E402
from ik_compare import compare, load, problem  # noqa: E402

HOME = np.array([2.811, -0.514, 0.102, -2.213, 0.051, 1.701, -1.033])
TOOL = {"T": [0.0, 0.0, 0.1034], "R": list(np.eye(3).ravel())}


def _rec(phase, q_true, seed, lula_q, tool=TOOL):
    tl = None if tool is None else (np.asarray(tool["T"]), np.asarray(tool["R"]).reshape(3, 3))
    p, R = ak.fk_tool(q_true, tl)
    return {"phase": phase, "frame": "right_gripper", "p_arm": p.tolist(), "R_arm": R.ravel().tolist(),
            "tool": tool, "seed": list(seed), "lula_ok": lula_q is not None,
            "lula_q": None if lula_q is None else list(lula_q)}


def test_가까운_씨앗이면_둘다_풀고_모델오차는_0이다():
    out = compare([_rec("approach", HOME, HOME + 0.05, HOME)])
    s = out["구간"]["approach"]
    assert s["둘다"] == 1 and s["모델오차mm"] < 1e-6 and s["ak오차mm"] < 1.0


def test_Lula해가_틀리면_모델문제로_판정된다():
    """Lula 해가 목표와 3 cm 어긋난 자세라면(다른 모델), 그 해를 씨앗으로 줘도 ak 는 거기 못 머문다."""
    q_wrong = HOME + np.array([0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0])
    far = HOME + np.array([1.5, 0.0, 1.2, 0.0, 1.0, 0.0, 0.0])          # ak 가 씨앗에서 못 풀게 먼 씨앗
    out = compare([_rec("carry_rotate", HOME, far, q_wrong)])
    s = out["구간"]["carry_rotate"]
    if s["Lula만"] == 1:                                                   # ak 가 먼 씨앗에서 실패한 경우만 판별
        assert out["판별"]["모델문제"] == 1
        assert out["못푼목표"][0]["Lula해_오차mm"] > 10.0
    assert s["모델오차mm"] > 10.0


def test_Lula해가_맞으면_씨앗문제로_판정된다():
    far = HOME + np.array([1.5, 0.0, 1.2, 0.0, 1.0, 0.0, 0.0])
    out = compare([_rec("carry_rotate", HOME, far, HOME)])
    s = out["구간"]["carry_rotate"]
    if s["Lula만"] == 1:
        assert out["판별"]["씨앗문제"] == 1
        assert out["못푼목표"][0]["Lula씨앗_ak_이동rad"] < 0.05
    assert s["모델오차mm"] < 1e-6


def test_덤프를_파일에서_읽고_문제로_되돌린다(tmp_path):
    f = tmp_path / "d.jsonl"
    f.write_text(json.dumps(_rec("lift", HOME, HOME, HOME)) + "\n\n" + json.dumps(_rec("down", HOME, HOME, None)) + "\n")
    recs = load(str(f))
    assert [r["phase"] for r in recs] == ["lift", "down"]
    p, R, tool, seed, lq = problem(recs[1])
    assert lq is None and tool is not None and R.shape == (3, 3)
    out = compare(recs)
    assert out["구간"]["down"]["ak만"] == 1                                  # Lula 못 품, ak 품
