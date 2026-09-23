"""서가 빈칸 찾기 단위시험 — Isaac 없이 돈다 (순수 기하).

실측 데이터는 `docs/doyoon-kim/measurements/20260923b_shelf_gaps.txt` 의
forthFloor(선반판 z 0.4976, 40권)에서 가져왔다.
"""

import os
import re
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

from shelf_gap import choose_gap, gaps, merge_boxes  # noqa: E402

#: 우리 책 두께 (m)
T = 0.0353

MEASURED = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "..", "..", "docs", "doyoon-kim", "measurements",
                        "20260923b_shelf_gaps.txt")


def _forth_floor():
    """실측 보고서에서 forthFloor 40권과 서가 안쪽 x 범위를 읽는다.

    표를 코드로 옮겨 적지 않는다 — 옮겨 적는 순간 실측과 갈라지고, 갈라진 것을
    아무도 모른다. 보고서가 저장소에 있으니 그것을 읽는다.
    """
    rows, inner, in_floor = [], None, False
    for line in open(MEASURED, encoding="utf-8"):
        if inner is None and "안쪽 x 범위" in line:
            m = re.findall(r"[-+]?\d*\.\d+", line)
            inner = (float(m[0]), float(m[1]))
        if line.startswith("② "):
            in_floor = "forthFloor" in line
            continue
        if not in_floor:
            continue
        m = re.match(r"\s+(\d+)\s+(\S+)\s+([-+]?\d*\.\d+)\s+([-+]?\d*\.\d+)", line)
        if m:
            rows.append((m.group(2), float(m.group(3)), float(m.group(4))))
    return rows, inner


FORTH, INNER = _forth_floor()


def test_the_fixture_is_the_measured_shelf():
    """실측 보고서를 그대로 읽었는가 — 40권, 안쪽 폭 1.4046 m."""
    assert len(FORTH) == 40
    assert INNER == pytest.approx((1.8692, 3.2738), abs=1e-4)


def test_the_real_gap_is_between_book20_and_the_pile():
    """실측이 말하는 59.9 mm 빈칸이 나온다 — 좌우 여유 12.3 mm."""
    g = [x for x in gaps(FORTH, *INNER) if abs(x.lo - 2.4665) < 1e-9]
    assert len(g) == 1
    assert g[0].width == pytest.approx(0.0599, abs=1e-4)
    assert g[0].center == pytest.approx(2.49645, abs=1e-5)
    assert g[0].clearance(T) * 1000 == pytest.approx(12.3, abs=0.1)
    assert g[0].left.endswith("book20") and "cover07" in g[0].right


def test_the_81mm_gap_is_not_real():
    """책 #29~#30 사이 81.1 mm 는 꽂을 수 있는 빈칸이 **아니다**.

    눕혀 쌓인 책(`cover15`)의 AABB 가 2.5691~3.1922 로 그 위를 통째로 덮는다.
    겹침 판정(`jam_report`)은 AABB 로 재므로 거기 꽂으면 그대로 겹친다.
    이웃한 두 권만 보고 세면 있는 것처럼 보인다 — 그래서 **묶어서** 본다.
    """
    assert not [x for x in gaps(FORTH, *INNER) if abs(x.lo - 2.8938) < 1e-9]
    blocks = merge_boxes(FORTH, 0.005)
    assert any(lo <= 2.8938 and hi >= 2.9749 for lo, hi, _n0, _n1 in blocks)


def test_edges_are_counted():
    """양 끝 옆판과의 틈도 빈칸이다 — 빠뜨려서 '빈칸이 없다' 고 잘못 본 적이 있다."""
    found = [round(g.width, 4) for g in gaps(FORTH, *INNER)]
    assert any(abs(w - 0.0460) < 1e-4 for w in found)   # 왼쪽 옆판
    assert any(abs(w - 0.0428) < 1e-4 for w in found)   # 오른쪽 옆판


def test_chooses_the_gap_we_were_aiming_at_not_the_widest():
    """넓은 칸이 아니라 **겨누던 자리에 가까운** 칸을 고른다."""
    g, why = choose_gap(gaps(FORTH, *INNER), want_x=2.4797, thickness=T)
    assert g is not None and g.center == pytest.approx(2.49645, abs=1e-5)
    assert "+16.8 mm" in why                            # 지금 꽂는 자리에서의 이동량


def test_moves_off_the_neighbour_we_were_digging_into():
    """9/24 새벽의 실제 값: 4.5 mm 파고들던 것이 12.3 mm 여유로 바뀐다."""
    # N1b2 는 book20 을 4.5 mm 파고들었다 → 우리 책 x최소 = 2.4665 − 0.0045
    now_center = (2.4665 - 0.0045) + T / 2
    g, _why = choose_gap(gaps(FORTH, *INNER), want_x=now_center, thickness=T)
    # 옮긴 뒤에는 book20 과 겹치지 않는다
    assert g.center - T / 2 > 2.4665
    assert (g.center - T / 2 - 2.4665) * 1000 == pytest.approx(12.3, abs=0.1)


def test_refuses_when_nothing_fits():
    """안 들어가면 **거절한다** — 임계를 낮추지 않는다."""
    g, why = choose_gap(gaps(FORTH, *INNER), want_x=2.48, thickness=0.20)
    assert g is None and "들어가는 빈칸이 없다" in why


def test_refuses_when_the_nearest_gap_is_far_away():
    """멀면 손대지 않는다 — 어느 칸을 겨냥한 명령인지 알 수 없다."""
    g, why = choose_gap(gaps(FORTH, *INNER), want_x=3.00, thickness=T, max_move=0.05)
    assert g is None and "떨어져 있다" in why


def test_touching_books_do_not_become_a_gap():
    """서로 −0.7~−4.6 mm 겹쳐 있는 책들 사이가 빈칸으로 잡히면 안 된다."""
    touching = [("a", 0.0, 0.10), ("b", 0.0975, 0.20), ("c", 0.2020, 0.30)]
    assert gaps(touching, join_below=0.005) == []
