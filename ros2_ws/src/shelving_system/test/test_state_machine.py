import pytest

from shelving_system.state_machine import (
    InvalidStateTransition,
    StateMachine,
    SystemState,
)


@pytest.fixture
def state_machine():
    """각 테스트에 새로운 FSM을 제공한다."""
    return StateMachine()


def move_to_next_book(state_machine):
    """FSM을 NEXT_BOOK 상태까지 정상적으로 진행시킨다."""
    transitions = [
        SystemState.IDLE,
        SystemState.PLANNING,
        SystemState.NAV_TO_RETURN,
        SystemState.RECEIVE_TRAY,
        SystemState.SELECT_BOOK,
        SystemState.NAV_TO_SHELF,
        SystemState.PLACE_BOOK,
        SystemState.UPDATE_DATA,
        SystemState.NEXT_BOOK,
    ]

    for next_state in transitions:
        state_machine.transition(next_state)


def test_initial_state_is_initializing(state_machine):
    """FSM 생성 직후 상태는 INITIALIZING이어야 한다."""
    assert state_machine.current_state == SystemState.INITIALIZING


def test_normal_flow_to_completed(state_machine):
    """책 한 권을 처리하고 HOME으로 복귀하는 정상 흐름을 검사한다."""
    transitions = [
        SystemState.IDLE,
        SystemState.PLANNING,
        SystemState.NAV_TO_RETURN,
        SystemState.RECEIVE_TRAY,
        SystemState.SELECT_BOOK,
        SystemState.NAV_TO_SHELF,
        SystemState.PLACE_BOOK,
        SystemState.UPDATE_DATA,
        SystemState.NEXT_BOOK,
        SystemState.RETURN_HOME,
        SystemState.COMPLETED,
        SystemState.IDLE,
    ]

    for next_state in transitions:
        state_machine.transition(next_state)
        assert state_machine.current_state == next_state

def test_nav_to_shelf_transitions_directly_to_place_book(state_machine):
    """서가 도착 후 상위 FSM은 바로 PLACE_BOOK으로 이동해야 한다."""
    transitions = [
        SystemState.IDLE,
        SystemState.PLANNING,
        SystemState.NAV_TO_RETURN,
        SystemState.RECEIVE_TRAY,
        SystemState.SELECT_BOOK,
        SystemState.NAV_TO_SHELF,
    ]

    for next_state in transitions:
        state_machine.transition(next_state)

    assert state_machine.can_transition(SystemState.PLACE_BOOK)
    state_machine.transition(SystemState.PLACE_BOOK)

    assert state_machine.current_state == SystemState.PLACE_BOOK


def test_next_book_can_continue_with_another_book(state_machine):
    """트레이에 책이 남아 있으면 NEXT_BOOK에서 SELECT_BOOK으로 돌아간다."""
    move_to_next_book(state_machine)

    assert state_machine.can_transition(SystemState.SELECT_BOOK)

    state_machine.transition(SystemState.SELECT_BOOK)

    assert state_machine.current_state == SystemState.SELECT_BOOK


def test_next_book_can_return_home(state_machine):
    """트레이에 책이 없으면 NEXT_BOOK에서 RETURN_HOME으로 이동한다."""
    move_to_next_book(state_machine)

    assert state_machine.can_transition(SystemState.RETURN_HOME)

    state_machine.transition(SystemState.RETURN_HOME)

    assert state_machine.current_state == SystemState.RETURN_HOME


def test_can_transition_does_not_change_current_state(state_machine):
    """can_transition은 가능 여부만 확인하고 현재 상태를 변경하지 않는다."""
    state_machine.transition(SystemState.IDLE)

    assert state_machine.can_transition(SystemState.PLANNING)
    assert not state_machine.can_transition(SystemState.PLACE_BOOK)
    assert state_machine.current_state == SystemState.IDLE


def test_invalid_transition_raises_exception(state_machine):
    """허용되지 않은 상태 전이는 예외를 발생시켜야 한다."""
    state_machine.transition(SystemState.IDLE)

    with pytest.raises(InvalidStateTransition):
        state_machine.transition(SystemState.PLACE_BOOK)

    assert state_machine.current_state == SystemState.IDLE


def test_failure_moves_machine_to_failed_state(state_machine):
    """작업 중 오류가 발생하면 FAILED 상태와 오류 정보를 저장해야 한다."""
    state_machine.transition(SystemState.IDLE)
    state_machine.transition(SystemState.PLANNING)

    state_machine.fail(
        error_code=1001,
        message="Failed to create a book placement plan.",
    )

    assert state_machine.current_state == SystemState.FAILED
    assert state_machine.error_code == 1001
    assert state_machine.error_message == (
        "Failed to create a book placement plan."
    )


def test_reset_moves_failed_machine_to_idle(state_machine):
    """실패한 FSM을 초기화하면 IDLE로 돌아가고 오류 정보가 제거되어야 한다."""
    state_machine.transition(SystemState.IDLE)
    state_machine.transition(SystemState.PLANNING)

    state_machine.fail(
        error_code=3001,
        message="Navigation timeout.",
    )

    state_machine.reset()

    assert state_machine.current_state == SystemState.IDLE
    assert state_machine.error_code == 0
    assert state_machine.error_message == ""


def test_completed_state_can_return_to_idle(state_machine):
    """작업 완료 후 다음 작업을 받기 위해 IDLE로 돌아갈 수 있어야 한다."""
    move_to_next_book(state_machine)

    state_machine.transition(SystemState.RETURN_HOME)
    state_machine.transition(SystemState.COMPLETED)

    assert state_machine.can_transition(SystemState.IDLE)

    state_machine.transition(SystemState.IDLE)

    assert state_machine.current_state == SystemState.IDLE