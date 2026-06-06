import unittest

from state_machine.actions import AGVAction
from state_machine.agv_state_machine import AGVStateMachine
from state_machine.events import AGVEvent
from state_machine.states import AGVState


class TestAGVStateMachine(unittest.TestCase):
    """Unit tests for the Phase 1 AGV state machine."""

    def test_initial_state_is_idle(self) -> None:
        """A new state machine should always start in IDLE."""
        state_machine = AGVStateMachine()

        self.assertEqual(state_machine.current_state, AGVState.IDLE)

    def test_idle_start_moves_forward(self) -> None:
        """IDLE + START should transition to MOVING and return MOVE_FORWARD."""
        state_machine = AGVStateMachine()

        action = state_machine.handle_event(AGVEvent.START)

        self.assertEqual(state_machine.current_state, AGVState.MOVING)
        self.assertEqual(action, AGVAction.MOVE_FORWARD)

    def test_moving_stop_returns_to_idle(self) -> None:
        """MOVING + STOP should transition to IDLE and return STOP."""
        state_machine = AGVStateMachine()
        state_machine.handle_event(AGVEvent.START)

        action = state_machine.handle_event(AGVEvent.STOP)

        self.assertEqual(state_machine.current_state, AGVState.IDLE)
        self.assertEqual(action, AGVAction.STOP)

    def test_moving_turn_requested_starts_turning(self) -> None:
        """MOVING + TURN_REQUESTED should transition to TURNING."""
        state_machine = AGVStateMachine()
        state_machine.handle_event(AGVEvent.START)

        action = state_machine.handle_event(AGVEvent.TURN_REQUESTED)

        self.assertEqual(state_machine.current_state, AGVState.TURNING)
        self.assertEqual(action, AGVAction.TURN_LEFT)

    def test_error_detected_from_idle_enters_error(self) -> None:
        """IDLE + ERROR_DETECTED should transition to ERROR and return STOP."""
        state_machine = AGVStateMachine()

        action = state_machine.handle_event(AGVEvent.ERROR_DETECTED)

        self.assertEqual(state_machine.current_state, AGVState.ERROR)
        self.assertEqual(action, AGVAction.STOP)

    def test_error_detected_from_moving_enters_error(self) -> None:
        """MOVING + ERROR_DETECTED should transition to ERROR and return STOP."""
        state_machine = AGVStateMachine()
        state_machine.handle_event(AGVEvent.START)

        action = state_machine.handle_event(AGVEvent.ERROR_DETECTED)

        self.assertEqual(state_machine.current_state, AGVState.ERROR)
        self.assertEqual(action, AGVAction.STOP)

    def test_error_detected_from_turning_enters_error(self) -> None:
        """TURNING + ERROR_DETECTED should transition to ERROR and return STOP."""
        state_machine = AGVStateMachine()
        state_machine.handle_event(AGVEvent.START)
        state_machine.handle_event(AGVEvent.TURN_REQUESTED)

        action = state_machine.handle_event(AGVEvent.ERROR_DETECTED)

        self.assertEqual(state_machine.current_state, AGVState.ERROR)
        self.assertEqual(action, AGVAction.STOP)

    def test_error_detected_from_error_stays_error(self) -> None:
        """ERROR + ERROR_DETECTED should stay in ERROR and return STOP."""
        state_machine = AGVStateMachine()
        state_machine.handle_event(AGVEvent.ERROR_DETECTED)

        action = state_machine.handle_event(AGVEvent.ERROR_DETECTED)

        self.assertEqual(state_machine.current_state, AGVState.ERROR)
        self.assertEqual(action, AGVAction.STOP)

    def test_unsupported_event_returns_no_action(self) -> None:
        """Unsupported combinations should not change state."""
        state_machine = AGVStateMachine()

        action = state_machine.handle_event(AGVEvent.STOP)

        self.assertEqual(state_machine.current_state, AGVState.IDLE)
        self.assertEqual(action, AGVAction.NO_ACTION)


if __name__ == "__main__":
    unittest.main()
