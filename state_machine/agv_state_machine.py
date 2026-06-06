from state_machine.actions import AGVAction
from state_machine.events import AGVEvent
from state_machine.states import AGVState


class AGVStateMachine:
    """Controls high-level AGV state transitions and action selection."""

    def __init__(self, initial_state: AGVState = AGVState.IDLE) -> None:
        """
        Create a new state machine.

        Args:
            initial_state: State used when the machine starts.
        """
        self.current_state = initial_state

    def handle_event(self, event: AGVEvent) -> AGVAction:
        """
        Process one event, update the current state, and return an action.

        Args:
            event: The event received by the state machine.

        Returns:
            The action that should be performed after this event.
        """
        # Error detection has the highest priority from every state.
        if event == AGVEvent.ERROR_DETECTED:
            self.current_state = AGVState.ERROR
            return AGVAction.STOP

        # IDLE + START -> MOVING + MOVE_FORWARD
        if self.current_state == AGVState.IDLE and event == AGVEvent.START:
            self.current_state = AGVState.MOVING
            return AGVAction.MOVE_FORWARD

        # MOVING + STOP -> IDLE + STOP
        if self.current_state == AGVState.MOVING and event == AGVEvent.STOP:
            self.current_state = AGVState.IDLE
            return AGVAction.STOP

        # MOVING + TURN_REQUESTED -> TURNING + TURN_LEFT
        if (
            self.current_state == AGVState.MOVING
            and event == AGVEvent.TURN_REQUESTED
        ):
            self.current_state = AGVState.TURNING
            return AGVAction.TURN_LEFT

        # SEARCHING + TAG_DETECTED -> ALIGNING + NO_ACTION
        if self.current_state == AGVState.SEARCHING and event == AGVEvent.TAG_DETECTED:
            self.current_state = AGVState.ALIGNING
            return AGVAction.NO_ACTION

        # ALIGNING + TAG_LOST -> SEARCHING + NO_ACTION
        if self.current_state == AGVState.ALIGNING and event == AGVEvent.TAG_LOST:
            self.current_state = AGVState.SEARCHING
            return AGVAction.NO_ACTION

        # ALIGNING + TAG_CHANGED -> ALIGNING + NO_ACTION
        if self.current_state == AGVState.ALIGNING and event == AGVEvent.TAG_CHANGED:
            self.current_state = AGVState.ALIGNING
            return AGVAction.NO_ACTION

        # Unsupported event/state combinations do not change the state.
        return AGVAction.NO_ACTION
