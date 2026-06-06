from state_machine.actions import AGVAction
from state_machine.agv_state_machine import AGVStateMachine
from core.application_state import ApplicationState
from core.event_manager import EventManager


class AGVApplication:
    """Connects application state, event handling, and the AGV state machine."""

    def __init__(self) -> None:
        """Create the main AGV application framework objects."""
        self.application_state = ApplicationState()
        self.event_manager = EventManager(self.application_state)
        self.state_machine = AGVStateMachine()
        self.is_running = False

    def initialize(self) -> None:
        """Prepare the application for processing events."""
        self.is_running = True
        self.application_state.current_state = self.state_machine.current_state
        self.application_state.pending_actions.clear()
        self.application_state.last_event = None

    def run_once(self) -> AGVAction:
        """
        Process one queued event and store the resulting action.

        Returns:
            The action returned by the state machine.
        """
        if not self.is_running:
            print(f"Action: {AGVAction.NO_ACTION.name}")
            return AGVAction.NO_ACTION

        event = self.event_manager.get_next_event()
        if event is None:
            print(f"Action: {AGVAction.NO_ACTION.name}")
            return AGVAction.NO_ACTION

        action = self.state_machine.handle_event(event)

        self.application_state.current_state = self.state_machine.current_state
        self.application_state.pending_actions.append(action)

        print(f"Action: {action.name}")
        return action

    def shutdown(self) -> None:
        """Stop the application framework from processing new events."""
        self.is_running = False
