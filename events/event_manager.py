from core.application_state import ApplicationState
from state_machine.events import AGVEvent


class EventManager:
    """Stores incoming events until the AGV application is ready to process them."""

    def __init__(self, application_state: ApplicationState) -> None:
        """Create an event manager connected to the shared application state."""
        self.application_state = application_state
        self._event_queue: list[AGVEvent] = []

    def add_event(self, event: AGVEvent) -> None:
        """Add a new event to the end of the queue."""
        self._event_queue.append(event)

    def has_events(self) -> bool:
        """Return True when at least one event is waiting in the queue."""
        return len(self._event_queue) > 0

    def get_next_event(self) -> AGVEvent | None:
        """Return the oldest waiting event, or None if the queue is empty."""
        if not self.has_events():
            return None

        event = self._event_queue.pop(0)
        self.application_state.last_event = event
        return event
