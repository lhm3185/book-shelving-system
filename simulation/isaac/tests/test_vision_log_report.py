"""비전 로그 보고의 산수 — 줄 읽기, y 묶기, ROI 여유, 1등의 흐름, 축별 분리."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))

from vision_log_report import (  # noqa: E402
    group_by_y, is_tray, parse_lines, pick_flow, report, roi_margin, separation, summarize)

LOG = """
   01:31:42 [비전] ROI 밖 책: (0.018, 0.545, 0.395) @arm_base_link
   01:31:42 [비전] ROI 밖 책 1권 제외 (남은 0권). ROI [-0.69, -0.2, 0.0] ~ [-0.26, 0.42, 0.32] @arm_base_link
   01:31:56 [비전] Book picked(best): xyz=(-0.337, 0.364, 0.210), center=(330, 320), conf=0.61, depth=patch, yaw=0.000 rad (후보 2권)
   01:31:59 [비전] Book picked(best): xyz=(-0.384, 0.415, 0.223), center=(300, 250), conf=0.78, depth=patch, yaw=0.000 rad (후보 2권)
   01:32:01 [비전] Book picked(best): xyz=(-0.384, 0.418, 0.222), center=(301, 251), conf=0.80, depth=patch, yaw=0.000 rad (후보 2권)
   01:32:03 [비전] Book picked(best): xyz=(-0.338, 0.365, 0.209), center=(331, 321), conf=0.63, depth=patch, yaw=0.000 rad (후보 2권)
   01:32:18 [조작] 책 관측이 안 정해진다: 관측 1개가 책 1권에 흩어져 있다 — 더 본다
"""
SLOTS = [-0.188, -0.004, 0.180, 0.364]


def _parsed():
    return parse_lines(LOG.splitlines())


def test_줄을_종류별로_읽는다():
    p = _parsed()
    assert len(p['picks']) == 4 and len(p['drops']) == 1 and len(p['node']) == 1
    assert p['picks'][0]['xyz'] == (-0.337, 0.364, 0.210)
    assert p['picks'][0]['center'] == (330.0, 320.0) and p['picks'][0]['candidates'] == 2
    assert p['roi'] == ((-0.69, -0.2, 0.0), (-0.26, 0.42, 0.32))


def test_유니코드_빼기표도_읽는다():
    p = parse_lines(['01:00:00 Book picked(best): xyz=(−0.432, 0.079, 0.057), center=(1, 2), conf=0.87'])
    assert p['picks'][0]['xyz'][0] == -0.432


def test_y_가_가까운_것끼리_묶는다():
    groups = group_by_y(_parsed()['picks'], 0.04)
    assert [len(g) for g in groups] == [2, 2]                 # 트레이 칸3 둘 · 끼어든 책 둘


def test_ROI_여유는_가장_가까운_면이고_밖이면_음수다():
    roi = ((-0.69, -0.2, 0.0), (-0.26, 0.42, 0.32))
    gap, face = roi_margin((-0.384, 0.415, 0.223), roi)
    assert abs(gap - 0.005) < 1e-9 and face == 'y 상한'       # 상한까지 5 mm
    gap, face = roi_margin((0.018, 0.545, 0.395), roi)
    assert gap < 0 and face == 'x 상한'                        # x 로 0.278 밖이 가장 크다


def test_칸에서_먼_무리는_트레이가_아니다():
    groups = group_by_y(_parsed()['picks'], 0.04)
    sums = [summarize(g, None, SLOTS) for g in groups]
    assert is_tray(sums[0], SLOTS, 0.05) and sums[0]['slot'][0] == 3
    assert not is_tray(sums[1], SLOTS, 0.05)                   # 칸3 에서 +52 mm


def test_1등의_흐름은_연속_구간으로_접힌다():
    picks = _parsed()['picks']
    groups = group_by_y(picks, 0.04)
    assert [(i, n) for i, n, _ in pick_flow(picks, groups)] == [(0, 1), (1, 2), (0, 1)]


def test_분리_여유는_겹치면_0_이고_신뢰도는_부호로_말한다():
    groups = group_by_y(_parsed()['picks'], 0.04)
    sums = [summarize(g, None, SLOTS) for g in groups]
    sep = separation([sums[0]], [sums[1]])
    assert abs(sep['y'][0] - (0.415 - 0.365)) < 1e-9           # y 로 50 mm 떨어져 있다
    assert abs(sep['center_b'][0] - (320 - 251)) < 1e-9        # 화면 둘째 좌표로 69
    assert sep['conf'][0] < 0                                  # 끼어든 책이 신뢰도로 이긴다


def test_보고서가_끼어든_무리를_짚는다():
    text = report(_parsed(), 0.04, SLOTS, 0.05)
    assert '트레이 칸이 아니다' in text and 'A×1' in text and 'B×2' in text


def test_빈_로그도_죽지_않는다():
    assert '뽑힌 검출 0개' in report(parse_lines([]), 0.04, SLOTS)


def test_3D_로_묶으면_y_가_가까운_다른_물체가_갈린다():
    from vision_log_report import group_by_xyz
    lines = [
        '[INFO] [1790611531.869153138] [vision_manager]: Book picked(best): xyz=(-0.274, 0.121, 0.154), center=(65, 496), conf=0.55',
        '[INFO] [1790611667.671437965] [vision_manager]: Book picked(best): xyz=(-0.452, 0.155, 0.198), center=(372, 249), conf=0.75',
        '[INFO] [1790611836.011241233] [vision_manager]: Book picked(best): xyz=(-0.451, 0.154, 0.193), center=(371, 250), conf=0.58',
    ]
    picks = parse_lines(lines)['picks']
    assert [len(g) for g in group_by_y(picks, 0.04)] == [3]            # y 로는 한 무리
    assert [len(g) for g in group_by_xyz(picks, 0.05)] == [1, 2]       # 3D 로는 둘
    assert all(p['time'] for p in picks)                               # epoch 시각도 읽는다
