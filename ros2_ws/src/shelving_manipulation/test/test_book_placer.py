"""PlaceTracker·MockSimExecutor 단위시험 (ROS 없이)."""

import pytest

from shelving_manipulation.book_placer import (
    cancel_command, decode, encode, ERRORS, MockSimExecutor, PHASES, PlaceTracker, progress_of,
    SIM_PHASE_ORDER, SIM_PHASE_TO_PHASE)


def tracker(**kw):
    args = {'token': 't1', 'job_id': 'j1', 'started_at': 0.0, 'heartbeat_timeout_s': 3.0,
            'goal_timeout_s': 60.0}
    args.update(kw)
    return PlaceTracker(**args)


def run_mock(mock, tr, until=30.0, dt=0.05, cancel_at=None):
    now = 0.0
    mock.handle({'type': 'place_book', 'token': tr.token, 'job_id': tr.job_id}, now)
    while now < until and tr.outcome is None:
        now += dt
        if cancel_at is not None and now >= cancel_at and tr.request_cancel():
            mock.handle(cancel_command(tr.token, tr.job_id), now)
        if tr.check(now) == 'cancel':
            mock.handle(cancel_command(tr.token, tr.job_id), now)
        tr.on_sim_state(mock.poll(now), now)
    return tr


def test_every_sim_phase_maps_to_team_phase():
    assert set(SIM_PHASE_ORDER) == set(SIM_PHASE_TO_PHASE)
    assert set(SIM_PHASE_TO_PHASE.values()) <= set(PHASES)
    # 결과 단계로 쓰이지 않는 DETECTING_BOOK 제외 전 단계가 한 번 이상 보고된다
    assert set(SIM_PHASE_TO_PHASE.values()) == set(PHASES) - {'DETECTING_BOOK'}


def test_error_codes_are_m4xx():
    # 413 INTERNAL_ERROR 는 2026-09-24 에 더했다 — 실행 콜백의 예외로 노드가 잠기던
    # 것을 **작업의 실패**로 끝내려면 그 자리를 가리킬 코드가 있어야 한다.
    assert sorted(ERRORS) == list(range(401, 414))


def test_success_reports_all_phases_and_monotonic_progress():
    tr = tracker()
    seen = []
    mock = MockSimExecutor(step_s=0.1)
    now = 0.0
    mock.handle({'type': 'place_book', 'token': 't1', 'job_id': 'j1'}, now)
    while tr.outcome is None:
        now += 0.05
        tr.on_sim_state(mock.poll(now), now)
        seen.append(tr.progress)
    assert tr.outcome.success and tr.outcome.placement_verified and tr.outcome.error_code == 0
    assert seen == sorted(seen) and seen[-1] == 1.0
    assert tr.history[0] == 'PLANNING_GRASP' and tr.history[-1] == 'VERIFYING'


def test_failure_keeps_code_and_phase():
    tr = run_mock(MockSimExecutor(step_s=0.1, fail_at='lift', fail_code=405), tracker())
    assert not tr.outcome.success
    assert tr.outcome.error_code == 405 and tr.outcome.failed_phase == 'GRASPING'


def test_failure_without_code_becomes_timeout():
    tr = tracker()
    tr.on_sim_state({'token': 't1', 'status': 'FAILED', 'phase': 'wedge'}, 1.0)
    assert tr.outcome.error_code == 404 and tr.outcome.failed_phase == 'INSERTING'


def test_unverified_placement_is_409():
    tr = run_mock(MockSimExecutor(step_s=0.05, unverified=True), tracker())
    assert tr.outcome.error_code == 409 and tr.outcome.failed_phase == 'VERIFYING'
    assert not tr.outcome.placement_verified


def test_cancel_waits_for_safe_stop_then_412():
    mock = MockSimExecutor(step_s=0.1)
    tr = run_mock(mock, tracker(), cancel_at=0.5)
    assert tr.outcome.error_code == 412
    # 시뮬이 CANCELLED 를 알리기 전에는 끝나지 않는다 (즉시 결과 반환 금지)
    assert mock.done['status'] == 'CANCELLED'


def test_goal_timeout_cancels_and_reports_404():
    tr = run_mock(MockSimExecutor(step_s=10.0), tracker(goal_timeout_s=1.0), until=40.0)
    assert tr.timed_out and tr.outcome.error_code == 404


def test_no_executor_is_not_ready_411():
    tr = tracker(heartbeat_timeout_s=1.0)
    assert tr.check(0.5) is None and tr.outcome is None
    tr.check(1.5)
    assert tr.outcome.error_code == 411


def test_executor_goes_silent_is_404():
    tr = tracker(heartbeat_timeout_s=1.0)
    tr.on_sim_state({'token': 't1', 'status': 'RUNNING', 'phase': 'approach'}, 0.2)
    tr.check(2.0)
    assert tr.outcome.error_code == 404 and tr.outcome.failed_phase == 'APPROACHING_BOOK'


def test_other_token_is_ignored():
    tr = tracker()
    stale = {'token': 'old', 'status': 'SUCCEEDED', 'placement_verified': True}
    assert not tr.on_sim_state(stale, 0.1)
    assert tr.outcome is None


def test_cancel_request_sent_once():
    tr = tracker()
    assert tr.request_cancel() and not tr.request_cancel()


def test_protocol_roundtrip_and_bad_input():
    msg = {'type': 'place_book', 'token': 'x', 'place': {'center': [1.0, 2.0, 3.0]}}
    assert decode(encode(msg)) == msg
    assert decode('not json') is None and decode('[1, 2]') is None


def test_progress_unknown_phase_is_zero():
    assert progress_of('nope') == 0.0


def test_failure_injection_not_skipped_by_slow_polling():
    mock = MockSimExecutor(step_s=0.01, fail_at='lift', fail_code=405)
    mock.handle({'type': 'place_book', 'token': 't1', 'job_id': 'j1'}, 0.0)
    state = mock.poll(5.0)          # 모든 동작 시간이 지난 뒤 첫 폴링
    assert state['status'] == 'FAILED' and state['phase'] == 'lift' and state['error_code'] == 405


# ------------------------------------------ 긴 동작 중의 취소 (2026-09-24 VD2)
#
#   시뮬 실행기의 회전은 스스로 수백 스텝을 돌린다. 그동안 명령을 받는 루프가
#   돌아오지 않아 **취소가 받아지지도 않았다.** 조작 노드가 30 초에 포기하고
#   취소를 보냈는데 시뮬은 418 초를 더 돌았다. 그래서 긴 동작이 직접 꺼내 본다.

def test_pick_cancel_finds_my_cancel():
    """내 토큰의 취소를 꺼내고, 꺼냈다고 말한다."""
    from shelving_manipulation.book_placer import cancel_command, encode, pick_cancel
    inbox = [encode(cancel_command('tok-A', 'job:rotate'))]
    keep, hit = pick_cancel(inbox, 'tok-A')
    assert hit and keep == []


def test_pick_cancel_ignores_someone_elses_cancel():
    """다른 토큰의 취소는 내 것이 아니다 — 꺼내지도, 멈추지도 않는다."""
    from shelving_manipulation.book_placer import cancel_command, encode, pick_cancel
    inbox = [encode(cancel_command('tok-B', 'job:rotate'))]
    keep, hit = pick_cancel(inbox, 'tok-A')
    assert not hit and keep == inbox


def test_pick_cancel_leaves_other_commands_alone():
    """**다른 명령은 건드리지 않는다.** 꺼내 버리면 제 차례에 처리될 것이 사라진다."""
    from shelving_manipulation.book_placer import cancel_command, encode, pick_cancel
    other = encode({'type': 'place', 'token': 'tok-C', 'job_id': 'j'})
    inbox = [other, encode(cancel_command('tok-A', 'j:rotate')), other]
    keep, hit = pick_cancel(inbox, 'tok-A')
    assert hit and keep == [other, other]


def test_pick_cancel_survives_garbage():
    """형식이 깨진 메시지가 섞여도 죽지 않는다 — 그건 그대로 두고 지나간다."""
    from shelving_manipulation.book_placer import pick_cancel
    keep, hit = pick_cancel(['{쓰레기', ''], 'tok-A')
    assert not hit and keep == ['{쓰레기', '']


# --------------------------------- 피드백이 작업을 죽이지 않게 (2026-09-24 노드 사망)
#
#   스택이 잘린 줄 알았는데 아니었다. `_execute → _feedback → publish_feedback` 이
#   **전체**였고 rclpy 가 RCLError 를 던진 자리가 바로 거기였다. 목표가 이미 끝났거나
#   노드가 내려가는 중이면 핸들이 무효가 되고, 그 예외가 `_execute` 밖으로 튀어
#   실행 스레드를 끝낸다. **피드백은 장식이다. 그것 때문에 죽으면 안 된다.**

class _Handle:
    def __init__(self, active=True, raises=None):
        self.is_active = active
        self.raises = raises
        self.sent = []

    def publish_feedback(self, msg):
        if self.raises is not None:
            raise self.raises
        self.sent.append(msg)


def test_feedback_goes_out_when_the_goal_is_alive():
    from shelving_manipulation.book_placer import publish_feedback_safely
    h = _Handle()
    ok, why = publish_feedback_safely(h, 'fb')
    assert ok and why == '' and h.sent == ['fb']


def test_a_finished_goal_is_not_published_to():
    """끝난 목표에 보내면 rclpy 가 던진다 — **보내기 전에 본다**."""
    from shelving_manipulation.book_placer import publish_feedback_safely
    h = _Handle(active=False)
    ok, why = publish_feedback_safely(h, 'fb')
    assert not ok and '이미 끝났다' in why and h.sent == []


def test_an_exception_is_swallowed_not_raised():
    """**여기서 터져도 작업은 계속된다.** 이게 노드를 죽이던 자리다."""
    from shelving_manipulation.book_placer import publish_feedback_safely
    h = _Handle(raises=RuntimeError('failed to publish'))
    ok, why = publish_feedback_safely(h, 'fb')       # 예외가 밖으로 안 나온다
    assert not ok and 'RuntimeError' in why


def test_the_reason_is_reported_not_hidden():
    """**조용히 삼키지 않는다.** 몇 번 못 보냈는지가 보여야 원인을 판다."""
    from shelving_manipulation.book_placer import publish_feedback_safely
    seen = []
    publish_feedback_safely(_Handle(raises=RuntimeError('boom')), 'fb', on_error=seen.append)
    assert seen and 'boom' in seen[0]


def test_a_handle_without_is_active_still_works():
    """`is_active` 가 없는 핸들(가짜 시뮬·옛 rclpy)도 막지 않는다."""
    from shelving_manipulation.book_placer import publish_feedback_safely

    class Bare:
        def __init__(self):
            self.sent = []

        def publish_feedback(self, msg):
            self.sent.append(msg)

    b = Bare()
    ok, _why = publish_feedback_safely(b, 'fb')
    assert ok and b.sent == ['fb']


# ------------------------- 사이클이 사이클을 망친다 (2026-09-24 생중계 확정 뒤)
#
#   지금까지 잰 조건: 판마다 시뮬·노드를 새로 띄우고 사이클 1번.
#   생중계 시연:      시뮬·노드를 한 번 띄우고 사이클을 여러 번.
#   **시연 조건은 한 번도 안 쟀다.** 그 조건에서만 나는 것이 이것이다.
#
#   빈칸은 스캔할 때 잰다. 그런데 책을 한 권 꽂으면 **그 빈칸이 없어진다.**
#   다시 안 재고 그 값을 쓰면 방금 채운 자리에 또 꽂는다.

def test_a_fresh_measurement_is_usable():
    from shelving_manipulation.book_placer import stale_gap_reason
    assert stale_gap_reason(0, 0) is None
    assert stale_gap_reason(3, 3) is None


def test_a_measurement_from_before_a_placement_is_refused():
    """**한 번만 꽂아도 낡는다.** 그 한 권이 빈칸을 채웠다."""
    from shelving_manipulation.book_placer import stale_gap_reason
    why = stale_gap_reason(0, 1)
    assert why is not None and '배치 1회 전' in why and '다시 스캔' in why


def test_the_reason_says_how_many_placements_ago():
    from shelving_manipulation.book_placer import stale_gap_reason
    assert '배치 3회 전' in stale_gap_reason(2, 5)


def test_never_measured_is_refused_too():
    """안 잰 것과 낡은 것은 다르지만, **둘 다 꽂으면 안 된다**."""
    from shelving_manipulation.book_placer import stale_gap_reason
    why = stale_gap_reason(None, 0)
    assert why is not None and '스캔이 먼저' in why


# ----------------------- 노드를 잠그지 않는다 (2026-09-24 생중계 대비)
#
#   실행 콜백에서 예외 하나가 밖으로 나가면 `_busy` 가 안 풀려 **그 뒤 모든 요청이
#   거절**됐다. 프로세스는 살아 있으니 죽은 것처럼 보이지도 않는다 — 증상은 "멈춤"
#   이 아니라 "거절" 이고, 다시 띄우기 전엔 복구가 없었다.
#
#   **작업 하나를 잃는 것과 노드를 잃는 것은 다르다.**

def test_an_internal_error_is_its_own_code():
    from shelving_manipulation.book_placer import error_name, internal_failure
    code, phase, _msg = internal_failure(NameError('x is not defined'))
    assert code == 413 and phase == 'INTERNAL'
    assert error_name(code) == 'INTERNAL_ERROR'


def test_the_message_names_the_exception():
    """**삼키는 것이 아니라 끝내는 것이다.** 무엇이 터졌는지가 남아야 한다."""
    from shelving_manipulation.book_placer import internal_failure
    _code, _phase, msg = internal_failure(NameError('publish_feedback_safely is not defined'))
    assert 'NameError' in msg and 'publish_feedback_safely' in msg


def test_the_message_says_the_node_keeps_going():
    """FSM 이 읽고 **재시도해도 된다**는 것을 알 수 있어야 한다."""
    from shelving_manipulation.book_placer import internal_failure
    assert '다음 요청은 받는다' in internal_failure(RuntimeError('boom'))[2]


def test_the_traceback_rides_along_when_given():
    """어디서 터졌는지가 안 남으면 다음에 또 못 고친다."""
    from shelving_manipulation.book_placer import internal_failure
    msg = internal_failure(RuntimeError('boom'), 'File "a.py", line 3, in f\n')
    assert 'a.py' in msg[2]
    assert msg[2].splitlines()[0].endswith('다음 요청은 받는다')   # 첫 줄만 써도 뜻이 선다


def test_the_new_code_is_retryable():
    """413 은 재시도 가능이어야 한다 — 노드가 다시 받을 수 있으니까."""
    from shelving_manipulation.book_placer import ERRORS
    assert ERRORS[413].retry is True


# ------------------- 들어가는 빈칸만 고른다 (2026-09-24 LIVE2 cycle2)
#
#   폭을 재 놓고 안 썼다. 첫 권을 꽂자 59.9 mm 빈칸이 12.5 / 11.0 두 조각이 됐는데,
#   다음 판이 **11 mm 조각의 중심이 검출에 제일 가깝다**는 이유로 그리로 끌어와
#   36.4 mm 책을 꽂았다 — 22.5 mm 겹쳤다(409).

T = 0.0364                      # 실측 꽂힌 가로 폭
FULL = [(-0.1065, -0.0466)]     # 폭 59.9 mm — 첫 권 꽂기 전
SPLIT = [(-0.1065, -0.0940), (-0.0583, -0.0473)]   # 12.5 · 11.0 mm — 꽂은 뒤


def test_the_full_gap_is_chosen_before_anything_is_placed():
    from shelving_manipulation.book_placer import choose_snap_gap
    gx, gw, why = choose_snap_gap(FULL, -0.0651, T)
    assert why is None
    assert gx == pytest.approx(-0.07655, abs=1e-4)
    assert gw * 1000 == pytest.approx(59.9, abs=0.1)


def test_the_leftover_slivers_are_refused():
    """**이것이 LIVE2 cycle2 다.** 두 조각 다 책보다 좁으면 꽂을 데가 없다."""
    from shelving_manipulation.book_placer import choose_snap_gap
    gx, gw, why = choose_snap_gap(SPLIT, -0.0651, T)
    assert gx is None and gw is None
    assert '들어가는 빈칸이 없다' in why
    assert '12.5 mm' in why          # 가장 넓은 조각을 사유에 싣는다
    assert '36.4 mm' in why          # 책 두께도 — 왜 안 되는지가 한 줄로 보인다


def test_a_fitting_gap_further_away_beats_a_sliver_nearby():
    """**가까운 조각보다 들어가는 칸이다.** 거리는 그 다음 기준이다."""
    from shelving_manipulation.book_placer import choose_snap_gap
    gaps = SPLIT + [(0.0200, 0.0800)]          # 60 mm 짜리가 멀리 하나
    gx, _gw, why = choose_snap_gap(gaps, -0.0651, T)
    assert why is None and gx == pytest.approx(0.05, abs=1e-6)


def test_a_far_fitting_gap_is_refused_when_a_limit_is_given():
    """멀면 손대지 않는다 — 어느 칸을 본 명령인지 알 수 없다."""
    from shelving_manipulation.book_placer import choose_snap_gap
    gaps = [(0.0200, 0.0800), (0.2000, 0.2600)]
    gx, _gw, why = choose_snap_gap(gaps, -0.0651, T, max_move=0.05)
    assert gx is None and '떨어져 있다' in why


def test_a_single_fitting_gap_ignores_the_distance_limit():
    """고를 것이 하나면 '어느 칸인지 모르겠다' 가 성립하지 않는다."""
    from shelving_manipulation.book_placer import choose_snap_gap
    gx, _gw, why = choose_snap_gap([(0.0200, 0.0800)], -0.0651, T, max_move=0.05)
    assert why is None and gx == pytest.approx(0.05, abs=1e-6)


def test_clearance_is_counted_on_both_sides():
    """폭이 두께와 같으면 **여유가 0 이다.** 그건 들어가는 게 아니다."""
    from shelving_manipulation.book_placer import choose_snap_gap
    exact = [(0.0, T)]
    assert choose_snap_gap(exact, 0.018, T, min_clearance=0.005)[0] is None
    assert choose_snap_gap(exact, 0.018, T, min_clearance=0.0)[0] is not None


# ------------- 프레임마다 흔들리는 책 관측 (2026-09-24 실측)
#
#   한 판 안에서 프레임마다 이렇게 나왔다:
#     x -0.344  y -0.025    ← 맞다
#     x -0.294  y -0.026    ← **같은 책인데 x 가 50 mm 튄다**
#     x -0.293  y +0.066    ← y 가 다르다 = **다른 책**
#   노드가 **마지막 프레임**을 썼고, 셋째로 집으러 갔다가 41 mm 어긋나 411.

FRAMES = [(-0.344, -0.025, 0.209), (-0.294, -0.026, 0.208), (-0.293, +0.066, 0.205)]


def test_it_does_not_just_take_the_last_frame():
    """**마지막 프레임을 쓰지 않는다.** 그게 41 mm 를 만들었다."""
    from shelving_manipulation.book_placer import steady_book
    x, y, _z, why = steady_book(FRAMES)
    assert (x, y) != (-0.293, 0.066)
    assert y == pytest.approx(-0.0255, abs=1e-3)      # 두 개짜리 묶음 쪽
    assert '책 2권' in why


def test_different_books_are_not_averaged_together():
    """**다른 책을 섞어 평균내면 둘 사이 허공이 나온다.** y 로 가른다."""
    from shelving_manipulation.book_placer import steady_book
    _x, y, _z, _why = steady_book(FRAMES)
    assert not (-0.025 > y > 0.066 or -0.025 < y < 0.066) or abs(y + 0.0255) < 1e-3


def test_the_median_survives_one_wild_frame():
    """한 프레임이 크게 튀어도 안 끌려간다 — 평균이 아니라 중앙값이다."""
    from shelving_manipulation.book_placer import steady_book
    pts = [(-0.334, 0.0, 0.2), (-0.336, 0.001, 0.2), (+0.500, 0.002, 0.2)]
    x, _y, _z, _why = steady_book(pts)
    assert x == pytest.approx(-0.334, abs=1e-6)


def test_a_single_observation_is_refused():
    """**한 프레임으로는 튄 것인지 알 수 없다.** 모르면서 집으러 가지 않는다."""
    from shelving_manipulation.book_placer import steady_book
    x, _y, _z, why = steady_book([(-0.294, 0.0, 0.2)])
    assert x is None and '알 수 없다' in why


def test_no_observation_says_so():
    from shelving_manipulation.book_placer import steady_book
    assert steady_book([])[0] is None


def test_the_reason_carries_the_numbers():
    """**왜 그 값인지가 남아야** 다음에 판정할 수 있다."""
    from shelving_manipulation.book_placer import steady_book
    _x, _y, _z, why = steady_book(FRAMES)
    assert '관측 3개' in why and 'mm' in why


# ------------------- 판을 가려서 맞춘다 (2026-09-24 두 권 판)
#
#   두 권째가 **같은 칸에** 또 꽂혔다. 빈칸 측정이 한 판만 했고, 꽂은 책도
#   목록에 없어서 폭이 59.8 mm 그대로였다. 그래서 막지도 못했다.
#   판을 여러 개 재면 이번엔 **층이 섞일** 수 있어서 판을 가려야 한다.

PITCH = 0.544        # 3·4번 선반 간격 (월드 1.042 − 0.498)
LOW, HIGH = 0.168, 0.168 + PITCH        # 빈칸 태그: 선반판 윗면 (팔 기준)
G = [[-0.1065, -0.0466, LOW], [0.0200, 0.0800, LOW], [0.1500, 0.2460, HIGH]]


def test_it_picks_only_the_gaps_on_that_board():
    from shelving_manipulation.book_placer import gaps_on_board
    same, rel = gaps_on_board(G, 0.0)                 # 아래 판 (상대 0)
    assert len(same) == 2 and rel == pytest.approx(0.0)
    same, rel = gaps_on_board(G, PITCH)               # 위 판
    assert len(same) == 1 and rel == pytest.approx(PITCH)


def test_absolute_height_is_not_compared():
    """**칸 중심과 판 윗면은 172 mm 다르다.** 상대 높이로 맞추니 기준이 달라도 된다."""
    from shelving_manipulation.book_placer import gaps_on_board
    # 노드 쪽: lower_z 0.3399 · middle_z 0.3399+PITCH → 상대는 0 과 PITCH
    assert len(gaps_on_board(G, (0.3399 + PITCH) - 0.3399)[0]) == 1
    assert len(gaps_on_board(G, 0.3399 - 0.3399)[0]) == 2


def test_a_height_between_boards_is_refused():
    """판 사이 허공이면 **빈 목록**을 준다 — 가까운 판으로 끌어당기지 않는다."""
    from shelving_manipulation.book_placer import gaps_on_board
    same, rel = gaps_on_board(G, PITCH / 2)
    assert same == [] and rel is not None


def test_untagged_gaps_fall_back_to_the_old_behaviour():
    """판이 안 붙은 옛 형식이면 전부 준다 — 한 판만 재던 때와 같다."""
    from shelving_manipulation.book_placer import gaps_on_board
    old = [[-0.1065, -0.0466], [0.02, 0.08]]
    same, rel = gaps_on_board(old, 0.0)
    assert same == old and rel is None


def test_the_second_book_cannot_reuse_the_filled_gap():
    """**두 권 판의 핵심.** 첫 권이 채운 칸은 조각이 되어 들어가는 칸에서 빠진다."""
    from shelving_manipulation.book_placer import choose_snap_gap, gaps_on_board
    filled = [[-0.1065, -0.0940, LOW], [-0.0583, -0.0473, LOW], [0.1500, 0.2460, HIGH]]
    same, _rel = gaps_on_board(filled, 0.0)
    gx, _gw, why = choose_snap_gap(same, -0.0651, 0.0352)
    assert gx is None and '들어가는 빈칸이 없다' in why      # 아래 판은 이제 못 쓴다
    same_hi, _rel = gaps_on_board(filled, PITCH)
    gx2, gw2, why2 = choose_snap_gap(same_hi, 0.198, 0.0352)
    assert why2 is None and gw2 * 1000 == pytest.approx(96.0, abs=0.1)   # 위 판 96 mm


# ---------------- 가리킨 판에 자리가 없으면 다른 판 (2026-09-24 두 권 판)
#
#   두 권째도 비전이 **아래 판**을 가리킨다(관측 z 0.400). 그 판은 첫 권이 채워
#   12/9 mm 조각만 남았고, **같은 순간 위 판에 56 mm 가 비어 있었다.**
#   가리킨 판만 보면 비어 있는 칸을 두고 거절한다 — 도윤님 목표가 3·4번 한 권씩이다.

FILLED_LOW = [[-0.1065, -0.0940, LOW], [-0.0583, -0.0473, LOW]]     # 12.5 / 11.0 mm
OPEN_HIGH = [[0.0980, 0.1540, HIGH]]                                 # 56 mm
BOTH = FILLED_LOW + OPEN_HIGH


def test_the_indicated_board_wins_when_it_fits():
    """**비전의 판단을 함부로 뒤집지 않는다.** 거기 들어가면 그걸로 끝이다."""
    from shelving_manipulation.book_placer import choose_gap_any_board
    gaps = [[-0.1065, -0.0466, LOW]] + OPEN_HIGH        # 아래 판 60 mm 비어 있다
    gx, gw, rel, why = choose_gap_any_board(gaps, -0.0651, 0.0, 0.0352)
    assert rel == pytest.approx(0.0) and gw * 1000 == pytest.approx(59.9, abs=0.1)
    assert '판을 옮긴다' not in (why or '')


def test_it_moves_to_the_other_board_when_the_indicated_one_is_full():
    """**이것이 두 권째다.** 아래 판은 조각뿐이고 위 판에 56 mm 가 있다."""
    from shelving_manipulation.book_placer import choose_gap_any_board
    gx, gw, rel, why = choose_gap_any_board(BOTH, -0.0651, 0.0, 0.0352)
    assert rel == pytest.approx(PITCH)                   # 위 판으로 갔다
    assert gw * 1000 == pytest.approx(56.0, abs=0.1)
    assert gx == pytest.approx(0.126, abs=1e-3)
    assert '판을 옮긴다' in why and '높이도 그 판 것으로' in why


def test_it_says_so_loudly_when_it_moves():
    """**판을 옮겼으면 말해야 한다** — 높이도 바꿔야 하므로 부르는 쪽이 알아야 한다."""
    from shelving_manipulation.book_placer import choose_gap_any_board
    _gx, _gw, _rel, why = choose_gap_any_board(BOTH, -0.0651, 0.0, 0.0352)
    assert '56 mm' in why and 'mm' in why


def test_the_widest_fitting_gap_wins_among_other_boards():
    """옮길 바에는 **가장 넓은** 칸으로 — 여유가 크고 가장 안전하다."""
    from shelving_manipulation.book_placer import choose_gap_any_board
    THIRD = HIGH + PITCH
    gaps = FILLED_LOW + [[0.0980, 0.1540, HIGH], [0.300, 0.396, THIRD]]   # 56 / 96
    _gx, gw, rel, _why = choose_gap_any_board(gaps, -0.0651, 0.0, 0.0352)
    assert gw * 1000 == pytest.approx(96.0, abs=0.1) and rel == pytest.approx(2 * PITCH)


def test_nowhere_to_go_is_refused():
    """**자리가 없으면 만들어내지 않는다**."""
    from shelving_manipulation.book_placer import choose_gap_any_board
    tiny = FILLED_LOW + [[0.10, 0.115, HIGH]]
    gx, _gw, _rel, why = choose_gap_any_board(tiny, -0.0651, 0.0, 0.0352)
    assert gx is None and '다른 판에도 들어가는 빈칸이 없다' in why


def test_untagged_gaps_still_work():
    """판 태그가 없으면 옛 동작 — 한 판만 재던 때와 같다."""
    from shelving_manipulation.book_placer import choose_gap_any_board
    gx, _gw, rel, _why = choose_gap_any_board([[-0.1065, -0.0466]], -0.0651, 0.0, 0.0352)
    assert gx is not None and rel is None
