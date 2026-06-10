from navigation.motion_commands import MotionCommand
from navigation.navigation_state import NavigationState
from navigation.route_manager import RouteManager


class NavigationStateManager:
    """Stateful navigation manager above RouteManager."""

    def __init__(self) -> None:
        """Create a navigation manager starting in WAIT_FOR_START."""
        self.state = NavigationState.WAIT_FOR_START
        self.route_manager = RouteManager()
        self.current_action: MotionCommand | None = None
        self.status_message = "Press F to start"
        self.last_processed_tag: int | None = None
        self.stop_before_turn_sent = False
        self.turn_right_sent = False

    def update(
        self,
        tag_id: int | None,
        start_pressed: bool = False,
        motion_complete: bool = False,
        alignment_complete: bool = False,
        tag_visible: bool = True,
    ) -> MotionCommand | None:
        """
        Update navigation state and return a command only on transitions.

        Args:
            tag_id: Currently visible AprilTag ID, or None.
            start_pressed: True when operator pressed F.
            motion_complete: True when the current motor motion is complete.
            alignment_complete: True when the turn tag is centered.
            tag_visible: True when the turn tag remains visible during alignment.

        Returns:
            MotionCommand to send once, otherwise None.
        """
        if self.state == NavigationState.WAIT_FOR_START:
            return self._handle_wait_for_start(tag_id, start_pressed)

        if self.state == NavigationState.MOVING:
            return self._handle_moving(tag_id)

        if self.state == NavigationState.PREPARE_TURN:
            return self._handle_prepare_turn(tag_id)

        if self.state == NavigationState.ALIGNING:
            return self._handle_aligning(alignment_complete, tag_visible)

        if self.state == NavigationState.TURNING:
            return self._handle_turning(tag_id, motion_complete)

        return None

    def _handle_wait_for_start(
        self,
        tag_id: int | None,
        start_pressed: bool,
    ) -> MotionCommand | None:
        """Handle startup state and manual F key start."""
        if not start_pressed:
            return None

        if tag_id != 1:
            self.status_message = "Cannot Start - Tag 1 Not Detected"
            print(self.status_message)
            return None

        self.state = NavigationState.MOVING
        self.last_processed_tag = 1
        self.current_action = MotionCommand.START_FORWARD
        self.status_message = "Moving"
        print("Detected Tag: 1")
        print("Executing START_FORWARD")
        return self.current_action

    def _handle_moving(self, tag_id: int | None) -> MotionCommand | None:
        """Transition from MOVING to PREPARE_TURN at Tag 2."""
        if tag_id != 2 or self.last_processed_tag == 2:
            return None

        self.state = NavigationState.PREPARE_TURN
        self.last_processed_tag = 2
        self.current_action = MotionCommand.START_SLOW_FORWARD
        self.status_message = "Prepare Turn"
        print("Detected Tag: 2")
        print("Executing START_SLOW_FORWARD")
        return self.current_action

    def _handle_prepare_turn(self, tag_id: int | None) -> MotionCommand | None:
        """Enter alignment when the turn marker Tag 3 is reached."""
        if tag_id != 3:
            return None

        if self.stop_before_turn_sent:
            return None

        self.state = NavigationState.ALIGNING
        self.status_message = "Aligning on Tag 3"
        print("Detected Tag: 3")
        print("Entering ALIGNING")
        return None

    def _handle_aligning(
        self,
        alignment_complete: bool,
        tag_visible: bool,
    ) -> MotionCommand | None:
        """
        Wait until the turn tag is centered before stopping for the turn.

        Args:
            alignment_complete: True when horizontal error is within threshold.
            tag_visible: True when Tag 3 is still visible.

        Returns:
            STOP once alignment is complete, otherwise None.
        """
        if not tag_visible:
            self.status_message = "Aligning - Tag 3 Lost"
            return None

        if not alignment_complete:
            self.status_message = "Aligning on Tag 3"
            return None

        self.state = NavigationState.TURNING
        self.stop_before_turn_sent = True
        self.current_action = MotionCommand.STOP
        self.status_message = "Stopping before turn"
        print("Alignment Complete")
        print("Executing STOP")
        return self.current_action

    def _handle_turning(
        self,
        tag_id: int | None,
        motion_complete: bool,
    ) -> MotionCommand | None:
        """Send TURN_RIGHT after STOP completes, then stop route at Tag 4."""
        if self.stop_before_turn_sent and not self.turn_right_sent:
            if not motion_complete:
                return None

            self.turn_right_sent = True
            self.last_processed_tag = 3
            self.current_action = MotionCommand.TURN_RIGHT
            self.status_message = "Turning Right"
            print("Executing TURN_RIGHT")
            return self.current_action

        if tag_id == 4 and self.last_processed_tag != 4:
            self.state = NavigationState.ROUTE_COMPLETE
            self.last_processed_tag = 4
            self.current_action = MotionCommand.STOP
            self.status_message = "Route Complete"
            print("Detected Tag: 4")
            print("Executing STOP")
            return self.current_action

        return None
