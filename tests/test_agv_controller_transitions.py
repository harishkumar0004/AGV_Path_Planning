import unittest

from state_machine.agv_state_machine import AGVStateMachine
from state_machine.events import AGVEvent
from state_machine.states import AGVState


class TestAGVControllerTransitions(unittest.TestCase):
    """Unit tests for perception-driven state transitions used by AGVController."""

    def test_tag_detected_moves_from_searching_to_aligning(self) -> None:
        """SEARCHING + TAG_DETECTED should transition to ALIGNING."""
        state_machine = AGVStateMachine(initial_state=AGVState.SEARCHING)

        state_machine.handle_event(AGVEvent.TAG_DETECTED)

        self.assertEqual(state_machine.current_state, AGVState.ALIGNING)

    def test_tag_lost_moves_from_aligning_to_searching(self) -> None:
        """ALIGNING + TAG_LOST should transition back to SEARCHING."""
        state_machine = AGVStateMachine(initial_state=AGVState.ALIGNING)

        state_machine.handle_event(AGVEvent.TAG_LOST)

        self.assertEqual(state_machine.current_state, AGVState.SEARCHING)

    def test_tag_changed_stays_aligning(self) -> None:
        """ALIGNING + TAG_CHANGED should stay in ALIGNING."""
        state_machine = AGVStateMachine(initial_state=AGVState.ALIGNING)

        state_machine.handle_event(AGVEvent.TAG_CHANGED)

        self.assertEqual(state_machine.current_state, AGVState.ALIGNING)


if __name__ == "__main__":
    unittest.main()
