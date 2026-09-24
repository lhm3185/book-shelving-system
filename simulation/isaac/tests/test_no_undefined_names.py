"""**부르는데 정의가 없는 이름**이 있는가 — Isaac 없이 도는 유일한 방어선.

2026-09-24 에 이것 때문에 시연 조건 측정 한 판(LIVE1, 6 사이클)을 통째로 버렸다.
`publish_feedback_safely` 를 부르는 코드는 넣었는데 **import 를 안 넣었다.** 결과는
첫 목표의 실행 콜백이 `NameError` 로 죽고, 작업이 '실행 중' 으로 걸린 채 남아
**그 뒤 모든 요청이 거절**되는 것이었다. 노드를 다시 띄우기 전까지 복구가 없다.

왜 안 잡혔나 — 두 가지가 겹쳤다:

1. **시험이 이 파일을 안 본다.** `manipulation_node` 의 액션 실행 콜백은 rclpy 와
   Isaac 이 있어야 도는데, 여기서는 둘 다 없다. 126개가 통과했지만 그 경로를 한 번도
   밟지 않았다. *"시험 통과" 가 옳다는 근거가 못 되는* 또 하나의 경우다.
2. **내 확인이 헛돌았다.** 패치 스크립트가 `assert "publish_feedback_safely" in text`
   로 확인했는데, **호출부 글자**가 그 조건을 만족시켜 버렸다. 넣으려던 것(import)이
   안 들어갔는데 확인은 통과했다.

그래서 **실행하지 않고도 잡을 수 있는 것은 실행하지 않고 잡는다.**
"""

import glob
import os

import pytest

pyflakes_api = pytest.importorskip(
    "pyflakes.api", reason="pyflakes 가 없다 — 이 방어선이 꺼진 채로 돈다")
from pyflakes.reporter import Reporter  # noqa: E402

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..")

#: 훑을 곳. **우리 것만** 본다 — 남의 패키지는 우리가 못 고친다
TREES = (
    "ros2_ws/src/shelving_manipulation/shelving_manipulation",
    "simulation/isaac/controllers",
    "simulation/isaac/tools",
)

#: 이미 있던 것. **새로 늘지만 않게 막는다** — 한 번에 다 고치려다 다른 걸 깨지 않는다
KNOWN = {
    ("simulation/isaac/tools/tray_fit.py", "_paths"),
}


class _Collect:
    """pyflakes 보고에서 **정의 없는 이름**만 줍는다."""

    def __init__(self):
        self.hits = []

    def unexpectedError(self, filename, msg):
        self.hits.append((filename, f"읽지 못했다: {msg}"))

    def syntaxError(self, filename, msg, lineno, offset, text):
        self.hits.append((filename, f"문법 오류 {lineno}: {msg}"))

    def flake(self, message):
        # UndefinedName / UndefinedLocal 만 본다. 안 쓰는 import 는 여기 관심사가 아니다
        if type(message).__name__ in ("UndefinedName", "UndefinedLocal",
                                      "UndefinedExport"):
            self.hits.append((message.filename, message.message_args[0]))


def _files():
    out = []
    for tree in TREES:
        out += sorted(glob.glob(os.path.join(ROOT, tree, "*.py")))
    return out


def test_there_are_files_to_check():
    """훑을 게 없는데 통과하면 안 된다 — 경로가 바뀌면 여기서 걸린다."""
    assert len(_files()) > 20


def test_nothing_calls_a_name_that_does_not_exist():
    """**정의 없는 이름을 부르는 곳이 없다.** LIVE1 을 버리게 한 그 종류다."""
    c = _Collect()
    for f in _files():
        pyflakes_api.checkPath(f, c)
    found = {(os.path.relpath(f, ROOT), name) for f, name in c.hits}
    new = found - KNOWN
    assert not new, "정의 없는 이름: " + "\n".join(f"  {f} — {n}" for f, n in sorted(new))


def test_the_known_list_does_not_rot():
    """고쳐 놓고 목록에 남겨 두면 **다음 것을 숨긴다.** 사라진 항목은 지운다."""
    c = _Collect()
    for f in _files():
        pyflakes_api.checkPath(f, c)
    found = {(os.path.relpath(f, ROOT), name) for f, name in c.hits}
    gone = KNOWN - found
    assert not gone, f"이미 고쳐졌다 — KNOWN 에서 지워라: {sorted(gone)}"
