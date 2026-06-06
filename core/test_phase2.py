import unittest

from state_machine.actions import AGVAction
from core.agv_application import AGVApplication
from state_machine.events import AGVEvent
from state_machine.states import AGVState


class TestPhase2AGVApplication(unittest.TestCase):
    """Unit tests for the Phase 2 AGV application framework."""

    def test_start_event_moves_application_to_moving(self) -> None:
        """START should produce MOVE_FORWARD and update shared state to MOVING."""
        application = AGVApplication()
        application.initialize()

        application.event_manager.add_event(AGVEvent.START)
        action = application.run_once()

        self.assertEqual(application.application_state.current_state, AGVState.MOVING)
        self.assertEqual(application.application_state.last_event, AGVEvent.START)
        self.assertEqual(action, AGVAction.MOVE_FORWARD)
        self.assertEqual(
            application.application_state.pending_actions,
            [AGVAction.MOVE_FORWARD],
        )

    def test_stop_event_moves_application_back_to_idle(self) -> None:
        """STOP after START should produce STOP and update shared state to IDLE."""
        application = AGVApplication()
        application.initialize()

        application.event_manager.add_event(AGVEvent.START)
        application.run_once()
        application.event_manager.add_event(AGVEvent.STOP)
        action = application.run_once()

        self.assertEqual(application.application_state.current_state, AGVState.IDLE)
        self.assertEqual(application.application_state.last_event, AGVEvent.STOP)
        self.assertEqual(action, AGVAction.STOP)
        self.assertEqual(
            application.application_state.pending_actions,
            [AGVAction.MOVE_FORWARD, AGVAction.STOP],
        )


if __name__ == "__main__":
    unittest.main()
