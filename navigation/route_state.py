from dataclasses import dataclass

from navigation.route_actions import RouteAction


@dataclass
class RouteState:
    """Stores persistent route execution state."""

    # True after Tag 0 starts the route.
    route_active: bool = False

    # True after the final stop tag is processed.
    route_completed: bool = False

    # Most recent tag that triggered a route action.
    last_processed_tag: int | None = None

    # Action prepared for a future route tag.
    upcoming_action: RouteAction | None = None

    # Index of the next expected tag in the fixed route.
    current_route_index: int = 0

    # True after Tag 3 requests a STOP before the right turn.
    waiting_for_turn_idle: bool = False

    # True after STOP has been sent for the Tag 3 turn sequence.
    turn_stop_sent: bool = False

    # Turn marker currently being handled.
    pending_turn_tag: int | None = None
