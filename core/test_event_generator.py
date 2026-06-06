from core.application_state import ApplicationState
from core.event_generator import EventGenerator


def print_events(title: str, events: list) -> None:
    """
    Print generated events in a beginner-friendly format.

    Args:
        title: Description of the state change being tested.
        events: Events returned by EventGenerator.
    """
    event_names = [event.name for event in events]
    print(f"{title}: {event_names}")


def main() -> None:
    """Demonstrate perception event generation from ApplicationState changes."""
    application_state = ApplicationState()
    event_generator = EventGenerator(application_state)

    # No tag -> tag visible produces TAG_DETECTED.
    application_state.perception.tag_visible = True
    application_state.perception.tag_id = 7
    application_state.perception.center_x = 320
    application_state.perception.center_y = 240
    events = event_generator.generate_events()
    print_events("No tag -> Tag 7 visible", events)

    # Same tag still visible produces no new event.
    application_state.perception.tag_visible = True
    application_state.perception.tag_id = 7
    application_state.perception.center_x = 330
    application_state.perception.center_y = 245
    events = event_generator.generate_events()
    print_events("Tag 7 still visible", events)

    # Visible tag ID changes produces TAG_CHANGED.
    application_state.perception.tag_visible = True
    application_state.perception.tag_id = 9
    application_state.perception.center_x = 300
    application_state.perception.center_y = 220
    events = event_generator.generate_events()
    print_events("Tag 7 -> Tag 9", events)

    # Tag visible -> no tag produces TAG_LOST.
    application_state.perception.tag_visible = False
    application_state.perception.tag_id = None
    application_state.perception.center_x = None
    application_state.perception.center_y = None
    events = event_generator.generate_events()
    print_events("Tag 9 visible -> No tag", events)


if __name__ == "__main__":
    main()
