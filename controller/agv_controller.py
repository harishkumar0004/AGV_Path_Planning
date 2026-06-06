from core.application_state import ApplicationState
from events.event_generator import EventGenerator
from events.event_types import PerceptionEvent
from perception.perception_manager import PerceptionManager
from state_machine.agv_state_machine import AGVStateMachine
from state_machine.events import AGVEvent
from state_machine.states import AGVState


class AGVController:
    """Connects perception, event generation, and state transitions."""

    def __init__(self) -> None:
        """Create all modules owned by the AGV controller."""
        self.application_state = ApplicationState()
        self.perception_manager = PerceptionManager(self.application_state)
        self.event_generator = EventGenerator(self.application_state)
        self.state_machine = AGVStateMachine(initial_state=AGVState.SEARCHING)
        self.is_initialized = False

    def initialize(self) -> bool:
        """
        Initialize all required controller modules.

        Returns:
            True if initialization succeeds, otherwise False.
        """
        camera_ready = self.perception_manager.initialize()
        if not camera_ready:
            return False

        self.application_state.current_state = self.state_machine.current_state
        self.is_initialized = True
        print(f"Current State: {self.state_machine.current_state.name}")
        return True

    def update(self) -> None:
        """
        Run one controller update cycle.

        This updates perception, generates perception events, sends those events
        to the state machine, and prints the resulting state transitions.
        """
        if not self.is_initialized:
            print("AGVController is not initialized.")
            return

        # PerceptionManager updates ApplicationState.perception internally.
        self.perception_manager.update()

        generated_events = self.event_generator.generate_events()
        if not generated_events:
            print(f"Current State: {self.state_machine.current_state.name}")
            return

        for perception_event in generated_events:
            state_machine_event = self._to_state_machine_event(perception_event)
            previous_state = self.state_machine.current_state

            self.state_machine.handle_event(state_machine_event)
            current_state = self.state_machine.current_state

            self.application_state.current_state = current_state
            self.application_state.last_event = state_machine_event

            print(f"Event: {perception_event.name}")
            print("Transition:")
            print(f"{previous_state.name} -> {current_state.name}")
            print(f"Current State: {current_state.name}")

    def shutdown(self) -> None:
        """Release controller resources safely."""
        self.perception_manager.release()
        self.is_initialized = False

    def _to_state_machine_event(self, perception_event: PerceptionEvent) -> AGVEvent:
        """
        Convert a perception event into a state-machine event.

        Args:
            perception_event: Event generated from ApplicationState perception data.

        Returns:
            Matching event understood by AGVStateMachine.
        """
        if perception_event == PerceptionEvent.TAG_DETECTED:
            return AGVEvent.TAG_DETECTED

        if perception_event == PerceptionEvent.TAG_LOST:
            return AGVEvent.TAG_LOST

        return AGVEvent.TAG_CHANGED
