"""필수·선택 Prim 검사 — 표에 튜플이 섞여 있어도 죽지 않는다.

2026-09-29: 선택 Prim 하나를 튜플(둘 중 하나)로 바꾸고 읽는 쪽을 안 고쳐, 시뮬이 시작하자마자
`Boost.Python.ArgumentError` 로 죽었다. 그 커밋 뒤에 오프라인 시험 250 개가 전부 통과했다 —
이 함수를 도는 시험이 없었기 때문이다. **기본 표 그대로** 도는 시험을 둔다.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config"))

world_loader = pytest.importorskip("world_loader")


class _Prim:
    def __init__(self, ok):
        self._ok = ok

    def IsValid(self):
        return self._ok


class _Stage:
    """진짜 stage 처럼 **문자열이 아니면 터진다** — 튜플을 그대로 넘기는 실수를 잡으려는 것이다."""

    def __init__(self, paths):
        self.paths = set(paths)

    def GetPrimAtPath(self, path):
        if not isinstance(path, str):
            raise TypeError(f"GetPrimAtPath 는 문자열을 받는다: {path!r}")
        return _Prim(path in self.paths)


def _first(value):
    return value if isinstance(value, str) else value[0]


def _last(value):
    return value if isinstance(value, str) else value[-1]


def test_기본_표_그대로_돈다_튜플의_어느_쪽이_있어도():
    said = []
    for pick in (_first, _last):
        stage = _Stage([pick(v) for v in world_loader.REQUIRED_PRIMS.values()]
                       + [pick(v) for v in world_loader.OPTIONAL_PRIMS.values()])
        world_loader.check_prims(stage, say=said.append)
    assert not any("선택 Prim 없음" in line for line in said)


def test_선택_Prim_이_없으면_알리기만_하고_필수가_없으면_멈춘다():
    said = []
    stage = _Stage([_first(v) for v in world_loader.REQUIRED_PRIMS.values()])
    world_loader.check_prims(stage, say=said.append)
    assert sum("선택 Prim 없음" in line for line in said) == len(world_loader.OPTIONAL_PRIMS)
    with pytest.raises(RuntimeError):
        world_loader.check_prims(_Stage([]), say=said.append)
