"""판정 도구 단위시험 — **사고 분류가 틀리면 완주율이 통째로 틀어진다**.

2026-09-24 에 실제로 그랬다. `previous crash` 를 기동 크래시로 셌더니 16판 전부가
"기동 크래시" 로 찍혔다. 그 줄은 크래시리포터가 시작할 때 **과거 덤프를 나열하는
줄**이고, 덤프가 39개 쌓여 있어서 멀쩡히 완주한 판마다 40줄씩 찍혔다.

세 줄 요약: **로그에 있는 글자가 이번 판의 사고를 뜻하지는 않는다.**
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))

from judge_run import incidents, load, read, verdict  # noqa: E402

#: 크래시리포터가 시작할 때마다 찍는 과거 덤프 목록. **사고가 아니다.**
PAST_DUMPS = "\n".join(
    f"[previous crash] preventing upload of minidump due to user opt-out: "
    f"'/home/x/isaacsim/kit/data/Kit/Isaac-Sim Python/5.1/{i:08x}-dead-beef.dmp'"
    for i in range(40))

#: 성공한 판의 sim.log 가 반드시 담는 줄
OK_SIM = PAST_DUMPS + "\n[Isaac] 씬 준비 완료 — 명령 대기\n"


def _run(tmp, **files):
    d = tmp / "R1"
    d.mkdir()
    for name, text in files.items():
        (d / name.replace("__", ".")).write_text(text, encoding="utf-8")
    return str(d)


# ----------------------------------------------------- 회귀: 오탐 그 자체
def test_past_crash_dumps_are_not_this_runs_crash(tmp_path):
    """**16/16 을 기동 크래시로 셌던 그 판.** 과거 덤프 40줄 + 정상 완주 = 사고 없음."""
    d = _run(tmp_path, sim__log=OK_SIM, cyc__log="code=0\ncycle rc=0\n")
    assert incidents(load(d))[0] == []


def test_the_false_positive_used_to_fail_a_passing_run(tmp_path):
    """그 오탐은 **합격 판정까지 뒤집었다** — 고친 뒤엔 합격이 합격으로 나온다."""
    d = _run(tmp_path,
             sim__log=OK_SIM + "한계 최소여유 0.208 rad\n[겹침] 옆 책과 겹치지 않음\n",
             cyc__log="code=0\ncycle rc=0\n")
    g = read(d)
    ok, bad, _warn = verdict(g)
    assert ok, bad
    assert g.get("incidents") is None


# ----------------------------------------------------- 기동
def test_boot_is_judged_only_by_the_ready_line(tmp_path):
    """기동의 자는 하나다 — `준비 완료` 가 있는가. 110판 6.4% 를 낼 때 쓴 기준이다."""
    d = _run(tmp_path, sim__log=PAST_DUMPS + "\nSegmentation fault\n")
    assert incidents(load(d))[0] == ["boot_crash"]


def test_a_run_with_no_sim_log_is_not_called_a_boot_crash(tmp_path):
    """로그가 아예 없는 것과 못 뜬 것은 다르다 — 없는 것을 사고로 세지 않는다."""
    d = _run(tmp_path, cyc__log="code=0\ncycle rc=0\n")
    assert incidents(load(d))[0] == []


def test_a_retry_that_worked_is_not_an_incident_but_is_still_said(tmp_path):
    """재시도로 떴으면 **감싸는 데 성공한 것**이다. 다만 한 번에 뜬 것은 아니다."""
    d = _run(tmp_path, sim__log=OK_SIM, sim__boot1__log="Segmentation fault\n",
             cyc__log="code=0\ncycle rc=0\n")
    inc, note = incidents(load(d))
    assert inc == []
    assert note["boot_retry"] == 2
    _ok, _bad, warn = verdict(read(d))
    assert any("2번째 시도" in w for w in warn)


# ----------------------------------------------------- PhysX
def test_physx_nan_is_its_own_class_and_carries_the_line_count(tmp_path):
    """28,202줄과 3줄은 다른 사실이다 — 줄 수를 같이 들고 다닌다."""
    d = _run(tmp_path, sim__log=OK_SIM + "Invalid PhysX transform\n" * 28202)
    inc, note = incidents(load(d))
    assert inc == ["physx_nan"] and note["physx_lines"] == 28202


# ----------------------------------------------------- 노드 사망
def test_shutdown_noise_after_a_finished_cycle_is_not_a_death(tmp_path):
    """사이클이 rc=0 으로 끝난 뒤의 rclpy 잡음은 사망이 아니다 — 종료 때 나는 소리다."""
    d = _run(tmp_path, sim__log=OK_SIM, cyc__log="code=0\ncycle rc=0\n",
             man__log="RCLError: failed to publish_feedback\n")
    assert incidents(load(d))[0] == []


def test_a_death_mid_cycle_is_an_incident(tmp_path):
    """VD1: 배치 전 단계에서 죽었다. 사이클이 결과를 못 냈다."""
    d = _run(tmp_path, sim__log=OK_SIM, cyc__log="cycle rc=124\n",
             man__log="  File a.py, line 1, in publish_feedback\nRCLError\n")
    assert incidents(load(d))[0] == ["node_died"]


# ----------------------------------------------------- 회전
def test_rotate_without_completion_is_a_hang(tmp_path):
    """VD2: 회전 명령 뒤 418초 멈췄다."""
    d = _run(tmp_path, sim__log=OK_SIM + "베이스 회전 명령 전송 90도\n", cyc__log="cycle rc=124\n")
    assert "rotate_hang" in incidents(load(d))[0]


def test_a_rotate_that_finished_is_not_a_hang(tmp_path):
    d = _run(tmp_path, sim__log=OK_SIM + "베이스 회전 명령 전송 90도\n베이스 회전 완료\n",
             cyc__log="code=0\ncycle rc=0\n")
    assert incidents(load(d))[0] == []


# ------------------------------------- 회전이 두 번이라는 것 (2026-09-24 VD2)
#
#   한 판에 회전이 **두 번** 있다 — 스캔 전, 꽂기 전. "완료가 하나도 없는가" 로
#   보면 첫 번째만 완료된 판이 통과한다. VD2 가 그래서 배치 실패로 잘못 찍혔다.

VD2_MAN = "베이스 회전 명령 전송 0도\n" * 2
VD2_SIM = ("베이스 회전 완료\n"
           "베이스 회전 시작: 현재 +90.0° → 목표 +0.0° "
           "(차이 -90.0°, 축 = 팔 베이스 [4.804, -5.282])\n주행 시간 초과\n")


def test_a_second_rotation_that_hung_is_still_a_hang(tmp_path):
    """첫 회전이 완료돼도 둘째가 멈췄으면 멈춘 것이다 — **수를 맞춰 본다.**"""
    d = _run(tmp_path, sim__log=OK_SIM + VD2_SIM, man__log=VD2_MAN, cyc__log="cycle rc=124\n")
    inc, note = incidents(load(d))
    assert "rotate_hang" in inc
    assert note["rotate_counts"] == (2, 1)


def test_the_hang_report_says_where_it_stopped(tmp_path):
    """**어디서 멈췄는지 같이 들고 나온다.** VD2 는 느린 게 아니라 엉뚱한 자리였다 —
    키오스크 [4.804, −5.282] 에서 90° 를 통째로 돌라는 명령이었다 (다른 판은 0°)."""
    d = _run(tmp_path, sim__log=OK_SIM + VD2_SIM, man__log=VD2_MAN, cyc__log="cycle rc=124\n")
    g = read(d)
    _ok, bad, _warn = verdict(g)
    assert any("4.804" in b and "-90.0" in b for b in bad), bad


def test_matched_rotations_are_not_a_hang(tmp_path):
    """명령 둘에 완료 둘이면 멈춘 게 아니다 — 정상 판을 사고로 세지 않는다."""
    d = _run(tmp_path, sim__log=OK_SIM + "베이스 회전 완료\n" * 2,
             man__log="베이스 회전 명령 전송 0도\n" * 2, cyc__log="code=0\ncycle rc=0\n")
    assert incidents(load(d))[0] == []


# ------------------------------------- 옆 실측 여유 (2026-09-24)
#
#   "겹침 0" 과 "여유가 있다" 는 다르다. 겹침 0 으로 통과한 판의 실제 여유가
#   한쪽 4.9 mm 였던 적이 있다 — 임계 5 mm 아래인데 판정은 합격이었다.

NOJAM = ("[겹침] 옆 책과 겹치지 않음 (그 판의 서가 책 40권과 대조) · "
         "좌 {l:+.1f} mm (book20) · 우 {r:+.1f} mm (cover07)\n")
PASS = "한계 최소여유 0.208 rad\n"


def _judged(tmp_path, left, right):
    d = _run(tmp_path, sim__log=OK_SIM + PASS + NOJAM.format(l=left, r=right),
             cyc__log="code=0\ncycle rc=0\n")
    g = read(d)
    return g, verdict(g)


def test_a_comfortable_run_says_nothing_extra(tmp_path):
    g, (ok, _bad, warn) = _judged(tmp_path, 12.5, 12.0)
    assert ok and g["side_min_mm"] == 12.0
    assert not any("옆 여유" in w for w in warn)


def test_a_thin_run_passes_but_is_said(tmp_path):
    """**합격을 뒤집지 않는다. 말만 한다.** 문턱을 올려 통과시키는 것과 반대다."""
    g, (ok, _bad, warn) = _judged(tmp_path, 4.9, 12.0)
    assert ok                                   # 겹침 0 이므로 합격이다
    assert g["side_min_mm"] == 4.9
    assert any("옆 여유" in w and "안전을 뜻하지 않는다" in w for w in warn)


def test_the_narrower_side_is_the_one_reported(tmp_path):
    g, _v = _judged(tmp_path, 12.5, 3.1)
    assert g["side_min_mm"] == 3.1


def test_a_run_without_the_line_is_not_guessed_at(tmp_path):
    """옛 판에는 그 줄이 없다 — 없으면 **없다고 둔다.** 0 으로 채우지 않는다."""
    d = _run(tmp_path, sim__log=OK_SIM + PASS + "[겹침] 옆 책과 겹치지 않음\n",
             cyc__log="code=0\ncycle rc=0\n")
    assert read(d).get("side_min_mm") is None


# ------------------------------ 눕혀 꽂힘 (2026-09-24 LIVE4)

POSE = "[꽂은 자세] 중심이 목표에서 {o:+.1f} mm · 가로 {sx:.1f} x {sy:.1f} mm → 수평 기울기 {d:+.2f}°\n"


def test_a_sideways_book_is_a_failure_not_a_warning(tmp_path):
    """85.8° 는 '조금 기울었다' 가 아니라 **누운 것**이다 — 합격을 준다면 틀렸다."""
    d = _run(tmp_path, sim__log=OK_SIM + PASS + NOJAM.format(l=49.4, r=49.4)
             + POSE.format(o=-6.3, sx=154.9, sy=45.0, d=85.80),
             cyc__log="code=0\ncycle rc=0\n")
    ok, bad, _warn = verdict(read(d))
    assert not ok
    assert any("눕혀 꽂혔다" in b for b in bad)


def test_a_small_tilt_is_still_only_a_warning(tmp_path):
    """1~30° 사이는 경고다 — 합격을 뒤집지 않는다. 둘을 섞으면 뜻이 없어진다."""
    d = _run(tmp_path, sim__log=OK_SIM + PASS + NOJAM.format(l=12.5, r=11.0)
             + POSE.format(o=1.5, sx=45.3, sy=152.0, d=3.57),
             cyc__log="code=0\ncycle rc=0\n")
    ok, _bad, warn = verdict(read(d))
    assert ok and any("기울기" in w for w in warn)


def test_the_normal_run_says_nothing_about_tilt(tmp_path):
    d = _run(tmp_path, sim__log=OK_SIM + PASS + NOJAM.format(l=12.5, r=11.0)
             + POSE.format(o=1.5, sx=36.4, sy=153.2, d=0.36),
             cyc__log="code=0\ncycle rc=0\n")
    ok, _bad, warn = verdict(read(d))
    assert ok and not any("기울기" in w or "눕혀" in w for w in warn)
