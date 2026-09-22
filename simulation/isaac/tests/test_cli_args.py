"""실행 스크립트가 **선언하지 않은 인자를 읽지 않는지** 본다. Isaac 없이 돈다.

왜 필요한가: `args.probe_tray` 를 쓰면서 `ap.add_argument("--probe-tray")` 를 빼먹어
`run_simulation.py` 가 **모든 실행에서 죽었다** (2026-09-22, a46c6db~). 파이썬은 이런 것을
실행할 때까지 못 잡고, Isaac 은 시작에 20초가 걸려 알아채는 데도 오래 걸렸다.

이 시험은 소스를 읽어 정적으로 비교하므로 1초도 안 걸린다.
"""

import os
import re

import pytest

ISAAC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
SCRIPTS = ("run_simulation.py", "tools/patrol.py", "tools/scan_shelf.py",
           "tools/pick_from_vision.py")

#: argparse 가 아닌 곳에서 오는 속성 (직접 넣어 준다)
EXTRA = {"run_simulation.py": {"usd"}}

#: 선언만 해 두고 읽지 않는 인자와 그 이유. **이유 없이 여기 넣지 말 것.**
KNOWN_UNUSED = {
    # `--gui` 의 반대말로 두었을 뿐, 코드는 `not args.gui` 만 본다.
    # 즉 `--headless` 를 줘도 아무 일도 하지 않는다 (기본이 이미 headless 라 무해하다).
    "run_simulation.py": {"headless"},
}


def _source(rel):
    with open(os.path.join(ISAAC, rel), encoding="utf-8") as fh:
        return fh.read()


def declared_args(src):
    """`ap.add_argument("--foo-bar")` → {"foo_bar"}"""
    out = set()
    # 따옴표는 작은/큰 둘 다 쓰인다
    for m in re.finditer(r"""add_argument\(\s*['"]--([a-zA-Z0-9-]+)['"]""", src):
        out.add(m.group(1).replace("-", "_"))
    return out


def used_args(src, holder="args"):
    """`args.foo_bar` → {"foo_bar"}

    **낱말 경계를 지킨다.** 그냥 "a." 로 찾으면 "data.get" 의 뒤쪽도 걸린다.
    """
    return set(re.findall(r"(?<![A-Za-z0-9_])" + holder + r"\.([a-z_][a-zA-Z0-9_]*)", src))


@pytest.mark.parametrize("rel", SCRIPTS)
def test_every_arg_read_is_declared(rel):
    """읽는 인자는 전부 선언돼 있어야 한다."""
    src = _source(rel)
    holder = "args" if "args = ap.parse_args()" in src else "a"
    missing = used_args(src, holder) - declared_args(src) - EXTRA.get(rel, set())
    assert not missing, f"{rel}: 선언 없이 읽는 인자 {sorted(missing)}"


@pytest.mark.parametrize("rel", SCRIPTS)
def test_declared_args_are_actually_used(rel):
    """선언만 해 두고 안 쓰는 인자는 없어야 한다 — 있으면 지우거나 붙이는 것을 잊은 것이다."""
    src = _source(rel)
    holder = "args" if "args = ap.parse_args()" in src else "a"
    unused = declared_args(src) - used_args(src, holder) - KNOWN_UNUSED.get(rel, set())
    assert not unused, (f"{rel}: 선언했지만 안 쓰는 인자 {sorted(unused)} — "
                        f"지우거나, 이유를 적어 KNOWN_UNUSED 에 넣을 것")
