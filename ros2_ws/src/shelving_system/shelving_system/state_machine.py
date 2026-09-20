"""State machine for the book shelving system."""

from enum import Enum, auto


class SystemState(Enum):
    """Top-level states of the book shelving system."""

    INITIALIZING = auto()
    IDLE = auto()
    PLANNING = auto()
    NAV_TO_RETURN = auto()
    RECEIVE_TRAY = auto()
    SELECT_BOOK = auto()
    NAV_TO_SHELF = auto()
    DETECT_TARGET_SLOT = auto()
    PLACE_BOOK = auto()
    UPDATE_DATA = auto()
    NEXT_BOOK = auto()
    RETURN_HOME = auto()
    COMPLETED = auto()
    WAIT_FOR_OPERATOR = auto()
    FAILED = auto()


class InvalidStateTransition(RuntimeError):
    """Raised when the FSM receives an invalid state transition."""

    def __init__(
        self,
        current_state: SystemState,
        requested_state: SystemState,
    ) -> None:
        """Initialize the exception with current and requested states."""
        self.current_state = current_state
        self.requested_state = requested_state

        message = (
            "Invalid state transition: "
            f"{current_state.name} -> {requested_state.name}"
        )

        super().__init__(message)


class StateMachine:
    """Manage the top-level workflow of the shelving system."""

    _ALLOWED_TRANSITIONS = {
        SystemState.INITIALIZING: {
            SystemState.IDLE,
            SystemState.FAILED,
            SystemState.WAIT_FOR_OPERATOR,
        },
        SystemState.IDLE: {
            SystemState.PLANNING,
            SystemState.FAILED,
        },
        SystemState.PLANNING: {
            SystemState.NAV_TO_RETURN,
            SystemState.FAILED,
            SystemState.WAIT_FOR_OPERATOR,
        },
        SystemState.NAV_TO_RETURN: {
            SystemState.RECEIVE_TRAY,
            SystemState.FAILED,
            SystemState.WAIT_FOR_OPERATOR,
        },
        SystemState.RECEIVE_TRAY: {
            SystemState.SELECT_BOOK,
            SystemState.FAILED,
            SystemState.WAIT_FOR_OPERATOR,
        },
        SystemState.SELECT_BOOK: {
            SystemState.NAV_TO_SHELF,
            SystemState.FAILED,
            SystemState.WAIT_FOR_OPERATOR,
        },
        SystemState.NAV_TO_SHELF: {
            SystemState.DETECT_TARGET_SLOT,
            SystemState.FAILED,
            SystemState.WAIT_FOR_OPERATOR,
        },
        SystemState.DETECT_TARGET_SLOT: {
            SystemState.PLACE_BOOK,
            SystemState.FAILED,
            SystemState.WAIT_FOR_OPERATOR,
        },
        SystemState.PLACE_BOOK: {
            SystemState.UPDATE_DATA,
            SystemState.FAILED,
            SystemState.WAIT_FOR_OPERATOR,
        },
        SystemState.UPDATE_DATA: {
            SystemState.NEXT_BOOK,
            SystemState.FAILED,
            SystemState.WAIT_FOR_OPERATOR,
        },
        SystemState.NEXT_BOOK: {
            SystemState.SELECT_BOOK,
            SystemState.RETURN_HOME,
            SystemState.FAILED,
            SystemState.WAIT_FOR_OPERATOR,
        },
        SystemState.RETURN_HOME: {
            SystemState.COMPLETED,
            SystemState.FAILED,
            SystemState.WAIT_FOR_OPERATOR,
        },
        SystemState.COMPLETED: {
            SystemState.IDLE,
        },
        SystemState.WAIT_FOR_OPERATOR: {
            SystemState.IDLE,
            SystemState.FAILED,
        },
        SystemState.FAILED: {
            SystemState.IDLE,
        },
    }

    def __init__(self) -> None:
        """Initialize the FSM in the INITIALIZING state."""
        self.current_state = SystemState.INITIALIZING
        self.error_code = 0
        self.error_message = ""

    def can_transition(self, next_state: SystemState) -> bool:
        """Return whether the requested transition is allowed."""
        allowed_states = self._ALLOWED_TRANSITIONS.get(
            self.current_state,
            set(),
        )

        return next_state in allowed_states

    def transition(self, next_state: SystemState) -> None:
        """Move to the requested state if the transition is allowed."""
        if not isinstance(next_state, SystemState):
            raise TypeError("next_state must be a SystemState value.")

        if not self.can_transition(next_state):
            raise InvalidStateTransition(
                current_state=self.current_state,
                requested_state=next_state,
            )

        self.current_state = next_state

    def fail(self, error_code: int, message: str) -> None:
        """Store error information and move the FSM to FAILED."""
        self.error_code = error_code
        self.error_message = message
        self.current_state = SystemState.FAILED

    def reset(self) -> None:
        """Clear error information and return the FSM to IDLE."""
        self.error_code = 0
        self.error_message = ""
        self.current_state = SystemState.IDLE
