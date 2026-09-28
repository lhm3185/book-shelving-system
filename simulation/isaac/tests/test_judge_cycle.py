"""풀 사이클 판정 — 줄 읽기와 판정 셋(합격 · 충돌 의심 · 불합격)."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))

from judge_cycle import parse, verdict  # noqa: E402

P = "2026-09-28T16:41:20Z [1ms] [Error] [omni.kit.app._impl] [py stderr]: ### "


def _place(t, c, k, ok=True, tail=""):
    body = ("SUCCEEDED {'phase': 'verify', 'message': '배치 확인', 'placement_verified': True, "
            "'checks': {'upright': True, 'depth': True, 'spine': True, 'x': True, 'floor': True, "
            "'skew': True, 'no_jam': True}}") if ok else tail
    return [
        f"2026-09-28T{t}Z [1ms] x: ### [꽂은 자세] 중심이 목표에서 {c:+.1f} mm · 가로 36 x 153 mm → 수평 기울기 {k:+.2f}°",
        f"2026-09-28T{t}Z [1ms] x: ### [겹침] 옆 책과 겹치지 않음 (그 판의 서가 책 43권과 대조) · 좌 +26.1 mm (A) · 우 +33.4 mm (B)",
        f"2026-09-28T{t}Z [1ms] x: ### 작업 job_1 {body}",
    ]


def _run(extra=(), books=4, end=True):
    lines = [P + "  팔 베이스 출발 자리 (+4.9850, -5.3071) yaw +90.00° — 돌아왔을 때 여기로 맞춘다",
             P + "  루트 출발 자리 (+4.9859, -5.6067) — **복귀는 여기로 간다**"]
    for i in range(books):
        lines += _place(f"16:4{i}:00", -2.0 - i * 0.5, 0.3)
    lines += list(extra)
    if end:
        lines += ["2026-09-28T16:52:42Z [1ms] x: ### [주행] 회전 끝 — **실측** yaw +90.00° (목표 +90.00°)",
                  "2026-09-28T16:52:44Z [1ms] x: ### [주행] 도착 (+4.986, -5.607)"]
    return parse(lines)


def test_네_권과_복귀가_맞으면_합격이다():
    facts = _run()
    assert len(facts['places']) == 4 and facts['places'][0]['jam']['against'] == 43
    assert verdict(facts) == ('합격', [])


def test_배치가_모자라거나_복귀를_안_했으면_불합격이다():
    assert verdict(_run(books=3))[0] == '불합격'
    result, why = verdict(_run(end=False))
    assert result == '불합격' and any('도착' in w or 'yaw' in w for w in why)


def test_꽂기는_통과했어도_뒤에서_403_이면_불합격이다():
    tail = ("FAILED {'error_code': 403, 'message': '관절 각속도 80% 초과 2스텝 (최대 98%)', "
            "'placement_verified': True, 'checks': {'upright': True, 'no_jam': True}}")
    facts = _run(books=3, extra=_place("16:49:00", -3.3, 0.37, ok=False, tail=tail))
    result, why = verdict(facts)
    assert result == '불합격' and any('403' in w for w in why)
    assert facts['places'][-1]['verified'] and facts['places'][-1]['status'] == 'FAILED'


def test_내려앉은_채_가면_충돌_의심이고_고쳤으면_합격이다():
    sag = "2026-09-28T16:48:22Z [1ms] x: ### [스윙] d(reorient) 처짐 152 mm · 손끝 0.674 → 최저 0.473 → 0.624 · **내려앉는다**"
    fix = "2026-09-28T16:48:23Z [1ms] x: ### [스윙] 바깥으로 뻗어 손목 돌리고 HORIZ 로 올라간다 — d 처짐 152 mm > 30 mm"
    assert verdict(_run(extra=[sag]))[0] == '충돌 의심'
    facts = _run(extra=[sag, fix])
    assert facts['sags'][0]['remedy'] == '되돌림 경로로 바꿨다' and verdict(facts)[0] == '합격'


def test_스윙_경고가_있으면_충돌_의심이다():
    warn = "2026-09-28T16:48:22Z [1ms] x: ### [스윙] **경고** 돌리는 구간 책 바닥 최저 0.415 가 바닥선 0.563 아래"
    assert verdict(_run(extra=[warn]))[0] == '충돌 의심'
