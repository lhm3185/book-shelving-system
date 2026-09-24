"""서가 빈칸 찾기 단위시험 — Isaac 없이 돈다 (순수 기하).

실측 데이터는 `docs/doyoon-kim/measurements/20260923b_shelf_gaps.txt` 의
forthFloor(선반판 z 0.4976, 40권)에서 가져왔다.
"""

import math
import os
import re
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers"))

from shelf_gap import (  # noqa: E402
    choose_gap, gaps, merge_boxes, skew_deg_from_span, span_for)

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


# ------------------------------------------------------- 비뚤어짐 (2026-09-24 +60 mm 판)
#
#   [겹침] book20 과 13.0 mm · 중심 +1.5 mm · x 폭 45.3 mm (부풀음 +10.1 mm) → 비뚤어짐
#
# 중심은 맞는데 폭이 부풀었다. upright(z)·spine(y)·x(중심) 어느 것도 못 잡는다.

DEPTH = 0.1631      # 책등→앞마구리


def test_skew_is_recovered_from_the_logged_span():
    """폭 45.3 mm 는 **3.6° 기울기**다 — 삽입 yaw 허용치 5.7° 안쪽이라 계획은 통과한다."""
    deg = skew_deg_from_span(0.0453, 0.0352, DEPTH)
    assert deg == pytest.approx(3.57, abs=0.05)
    assert deg < math.degrees(0.10)          # yaw_tolerance 0.10 rad = 5.73°


def test_a_thin_book_eats_a_lot_when_it_leans():
    """두께 35 mm 가 3.6° 돌면 45 mm 를 먹는다 — 지렛대는 책등→앞마구리 163 mm 다."""
    assert span_for(0.0352, DEPTH, 0.0) == pytest.approx(0.0352, abs=1e-9)
    assert span_for(0.0352, DEPTH, 3.57) * 1000 == pytest.approx(45.3, abs=0.2)
    # 두께만 보면 10 mm 여유가 넉넉해 보이지만 각도로는 3.5° 밖에 안 된다
    assert skew_deg_from_span(0.0452, 0.0352, DEPTH) == pytest.approx(3.53, abs=0.05)


def test_the_gap_tolerates_less_skew_than_it_looks():
    """여유 12.3 mm 인 빈칸이 견디는 기울기는 8.9° 뿐이다 — mm 로만 보면 놓친다."""
    g = [x for x in gaps(FORTH, *INNER) if abs(x.lo - 2.4665) < 1e-9][0]
    assert g.clearance(0.0352) * 1000 == pytest.approx(12.35, abs=0.1)
    assert g.max_skew_deg(0.0352, DEPTH) == pytest.approx(8.86, abs=0.05)
    # 왼쪽 옆판 빈칸(여유 5.4 mm)은 **3.8° 까지**다. +60 mm 판에서 실제로 난 기울기가
    # 3.57° 였으니 예비가 0.3° 뿐이다 — 이 칸은 사실상 기울기 예비가 없다
    left = [x for x in gaps(FORTH, *INNER) if x.left == ""][0]
    assert left.max_skew_deg(0.0352, DEPTH) == pytest.approx(3.82, abs=0.05)
    assert left.max_skew_deg(0.0352, DEPTH) - 3.57 < 0.3


# ------------------------------------------- 좌우 실측 여유 (2026-09-24 데스크탑 요청)
#
#   "겹침 0" 이 "여유가 있다" 를 뜻하지 않는다. 겹침 0 으로 통과한 판의 실제 여유가
#   한쪽 4.9 mm 였던 적이 있다(임계 5 mm). 통과와 아슬아슬함을 가르려면 숫자가 있어야
#   하는데, 지금 로그에는 겹쳤을 때의 침투량만 있고 안 겹쳤을 때의 여유가 없다.

def _box(x0, x1, y=(0.0, 0.16), z=(0.50, 0.73)):
    return (x0, y[0], z[0], x1, y[1], z[1])


OURS = _box(2.4790, 2.5142)        # 폭 35.2 mm 짜리 책 한 권


def test_clearance_is_measured_on_both_sides():
    """좌우 각각 가장 가까운 이웃까지의 거리를 낸다."""
    from shelf_gap import side_clearances
    boxes = [("left_book", _box(2.4600, 2.4665)), ("right_book", _box(2.5262, 2.5600))]
    lo, ln, ro, rn = side_clearances(OURS, boxes)
    assert lo * 1000 == pytest.approx(12.5, abs=0.1) and ln == "left_book"
    assert ro * 1000 == pytest.approx(12.0, abs=0.1) and rn == "right_book"


def test_the_nearest_neighbour_wins_not_the_first_one():
    """더 가까운 책이 뒤에 나와도 그쪽을 쓴다 — 순서에 기대지 않는다."""
    from shelf_gap import side_clearances
    boxes = [("far", _box(2.3000, 2.4000)), ("near", _box(2.4600, 2.4665))]
    lo, ln, _ro, _rn = side_clearances(OURS, boxes)
    assert ln == "near" and lo * 1000 == pytest.approx(12.5, abs=0.1)


def test_a_book_on_another_shelf_is_not_a_neighbour():
    """**x 만 보면 아래 칸 책이 이웃으로 잡힌다.** z 가 안 겹치면 옆이 아니다."""
    from shelf_gap import side_clearances
    below = [("lower_shelf", _box(2.4600, 2.4665, z=(0.20, 0.43)))]
    lo, _ln, ro, _rn = side_clearances(OURS, below)
    assert lo is None and ro is None


def test_a_book_behind_is_not_a_neighbour():
    """뒤쪽에 있는 책도 옆이 아니다 — y 도 본다."""
    from shelf_gap import side_clearances
    behind = [("back_row", _box(2.4600, 2.4665, y=(0.30, 0.46)))]
    assert side_clearances(OURS, behind)[0] is None


def test_no_neighbour_on_a_side_is_said_as_none():
    """이웃이 없으면 0 이 아니라 **없음**이다. 0 으로 쓰면 '딱 붙었다' 로 읽힌다."""
    from shelf_gap import side_clearances
    lo, _ln, ro, _rn = side_clearances(OURS, [("right_book", _box(2.5262, 2.5600))])
    assert lo is None and ro is not None


def test_the_tight_run_would_have_been_visible():
    """한쪽 4.9 mm 였던 판 — 겹침은 0 인데 여유는 임계(5 mm) 아래다.

    이 줄이 있었으면 "겹침 0 이라 안전하다" 고 읽지 않았을 것이다.
    """
    from shelf_gap import side_clearances
    tight = [("book20", _box(2.4600, 2.4741)), ("cover07", _box(2.5191, 2.5600))]
    lo, _ln, ro, _rn = side_clearances(OURS, tight)
    assert lo * 1000 == pytest.approx(4.9, abs=0.1)
    assert ro * 1000 == pytest.approx(4.9, abs=0.1)
    assert min(lo, ro) * 1000 < 5.0        # ← 임계 아래인데 겹침은 0 이다


# ----------------------------- 장면 조사의 자리 판정 (2026-09-24 LIVE2)
#
#   한 사이클만 성공했는데 조사가 "서가 2권" 이라고 했다. 판정이 **밑면 z 하나**
#   였기 때문이다 — 서가에서 한참 떨어져 있어도 그 높이면 "서가" 로 센다.

SHELF_Z, TRAY_Z = 0.4976, 0.3785
SHELF_XY = (1.8692, 3.2738, -2.57, -2.27)


def _at(x, y, z):
    return (x - 0.018, y - 0.076, z, x + 0.018, y + 0.076, z + 0.227)


def test_a_book_in_the_shelf_is_the_shelf():
    from shelf_gap import classify_place
    assert classify_place(_at(2.50, -2.42, SHELF_Z), SHELF_Z, TRAY_Z, SHELF_XY) == "서가"


def test_a_book_in_the_tray_is_the_tray():
    from shelf_gap import classify_place
    assert classify_place(_at(2.10, -3.05, TRAY_Z), SHELF_Z, TRAY_Z, SHELF_XY) == "트레이"


def test_shelf_height_but_far_away_is_not_the_shelf():
    """**이게 LIVE2 를 설명하는 판정이다.** 높이만 맞고 자리는 딴 데다."""
    from shelf_gap import classify_place
    held = _at(2.50, -3.00, SHELF_Z)          # 서가 앞 58 cm — 손에 들린 높이
    assert classify_place(held, SHELF_Z, TRAY_Z, SHELF_XY) == "서가높이·서가밖"


def test_without_the_shelf_box_the_old_answer_comes_back():
    """서가 상자를 모르면 옛 판정 그대로다 — **없는 정보로 단정하지 않는다.**"""
    from shelf_gap import classify_place
    held = _at(2.50, -3.00, SHELF_Z)
    assert classify_place(held, SHELF_Z, TRAY_Z, None) == "서가"


def test_the_floor_is_neither():
    from shelf_gap import classify_place
    assert classify_place(_at(2.5, -3.0, 0.01), SHELF_Z, TRAY_Z, SHELF_XY) == "바닥/기타"


def test_the_tray_wins_when_heights_are_close():
    """트레이를 먼저 본다 — 두 높이가 가까우면 트레이 쪽이 좁은 조건이다."""
    from shelf_gap import classify_place
    assert classify_place(_at(2.1, -3.05, TRAY_Z + 0.01), SHELF_Z, TRAY_Z, SHELF_XY) == "트레이"
