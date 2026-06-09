from navigation.route_actions import RouteAction
from navigation.route_state import RouteState


class RouteManager:
    """Maintains route state for a fixed AprilTag sequence."""

    def __init__(self) -> None:
        """Create a route manager for the fixed 1 -> 2 -> 3 -> 4 route."""
        self.route = [1, 2, 3, 4]
        self.state = RouteState()

    def process_tag(self, tag_id: int | None) -> RouteAction | None:
        """
        Process one detected tag and return a route action when needed.

        Args:
            tag_id: Detected AprilTag ID, or None if no tag is visible.

        Returns:
            RouteAction for newly processed route tags, otherwise None.
        """
        if tag_id is None:
            return None

        if self.state.route_completed:
            return None

        # Prevent repeated triggering while the same tag remains visible.
        if tag_id == self.state.last_processed_tag:
            return None

        if not self._is_expected_tag(tag_id):
            return None

        print(f"Detected Tag: {tag_id}")

        if tag_id == 1:
            return self._start_route(tag_id)

        if tag_id == 2:
            return self._prepare_turn(tag_id)

        if tag_id == 3:
            return self._execute_turn(tag_id)

        if tag_id == 4:
            return self._stop_route(tag_id)

        return None

    def _is_expected_tag(self, tag_id: int) -> bool:
        """
        Check whether the tag matches the next expected route tag.

        Args:
            tag_id: Detected AprilTag ID.

        Returns:
            True if tag_id is the next tag in the fixed route.
        """
        if self.state.current_route_index >= len(self.route):
            return False

        return tag_id == self.route[self.state.current_route_index]

    def _advance_route(self, tag_id: int) -> None:
        """
        Mark a tag as processed and move to the next route index.

        Args:
            tag_id: Tag that has just been processed.
        """
        self.state.last_processed_tag = tag_id
        self.state.current_route_index += 1

    def _start_route(self, tag_id: int) -> RouteAction:
        """
        Start the route at Tag 0.

        Args:
            tag_id: Detected start tag.

        Returns:
            FORWARD action.
        """
        self.state.route_active = True
        self._advance_route(tag_id)
        print("Executing FORWARD")
        return RouteAction.FORWARD

    def _continue_route(self, tag_id: int) -> RouteAction:
        """
        Continue forward route motion at Tag 1.

        Args:
            tag_id: Detected continuation tag.

        Returns:
            FORWARD action.
        """
        self._advance_route(tag_id)
        print("Executing FORWARD")
        return RouteAction.FORWARD

    def _prepare_turn(self, tag_id: int) -> RouteAction:
        """
        Prepare for the upcoming right turn at Tag 2.

        Args:
            tag_id: Detected preparation tag.

        Returns:
            FORWARD action while upcoming_action is set to TURN_RIGHT.
        """
        self.state.upcoming_action = RouteAction.TURN_RIGHT
        self._advance_route(tag_id)
        print("Upcoming Action: TURN_RIGHT")
        print("Executing FORWARD")
        return RouteAction.FORWARD

    def _execute_turn(self, tag_id: int) -> RouteAction:
        """
        Execute the prepared right turn at Tag 3.

        Args:
            tag_id: Detected turn tag.

        Returns:
            TURN_RIGHT action.
        """
        self._advance_route(tag_id)
        self.state.upcoming_action = None
        print("Executing TURN_RIGHT")
        return RouteAction.TURN_RIGHT

    def _stop_route(self, tag_id: int) -> RouteAction:
        """
        Stop and complete the route at Tag 4.

        Args:
            tag_id: Detected final stop tag.

        Returns:
            STOP action.
        """
        self._advance_route(tag_id)
        self.state.route_active = False
        self.state.route_completed = True
        self.state.upcoming_action = None
        print("Executing STOP")
        return RouteAction.STOP
