"""PlaceTracker·MockSimExecutor 단위시험 (ROS 없이)."""

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
    assert sorted(ERRORS) == list(range(401, 413))


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
