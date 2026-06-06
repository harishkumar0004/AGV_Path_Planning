from dataclasses import replace

from core.application_state import ApplicationState
from core.event_types import PerceptionEvent
from core.perception_state import PerceptionState


class EventGenerator:
    """Generates perception events by comparing old and new perception state."""

    def __init__(self, application_state: ApplicationState) -> None:
        """
        Create an event generator connected to shared application state.

        Args:
            application_state: Shared state that contains latest perception data.
        """
        self.application_state = application_state
        self.previous_perception_state = replace(application_state.perception)

    def generate_events(self) -> list[PerceptionEvent]:
        """
        Compare previous perception state with current state and return events.

        Returns:
            A list of perception events generated from the latest state change.
        """
        current_state = self.application_state.perception
        previous_state = self.previous_perception_state
        events: list[PerceptionEvent] = []

        # No tag before, tag visible now.
        if not previous_state.tag_visible and current_state.tag_visible:
            events.append(PerceptionEvent.TAG_DETECTED)

        # Tag visible before, no tag visible now.
        if previous_state.tag_visible and not current_state.tag_visible:
            events.append(PerceptionEvent.TAG_LOST)

        # A tag is still visible, but the ID changed.
        if self._tag_id_changed(previous_state, current_state):
            events.append(PerceptionEvent.TAG_CHANGED)

        # Store a copy of current state for the next comparison.
        self.previous_perception_state = replace(current_state)
        return events

    def _tag_id_changed(
        self,
        previous_state: PerceptionState,
        current_state: PerceptionState,
    ) -> bool:
        """
        Check whether the visible AprilTag ID changed.

        Args:
            previous_state: Perception state from the previous update.
            current_state: Latest perception state.

        Returns:
            True if both states have visible tags but different IDs.
        """
        if not previous_state.tag_visible or not current_state.tag_visible:
            return False

        return previous_state.tag_id != current_state.tag_id
