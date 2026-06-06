from dataclasses import dataclass, field

from core.perception_state import PerceptionState
from state_machine.actions import AGVAction
from state_machine.events import AGVEvent
from state_machine.states import AGVState


@dataclass
class ApplicationState:
    """Stores the shared high-level state of the AGV application."""

    # The current state reported by the state machine.
    current_state: AGVState = AGVState.IDLE

    # The most recent event processed by the application.
    last_event: AGVEvent | None = None

    # Actions requested by the state machine and waiting for execution.
    pending_actions: list[AGVAction] = field(default_factory=list)

    # The latest perception result shared with the rest of the application.
    perception: PerceptionState = field(default_factory=PerceptionState)
