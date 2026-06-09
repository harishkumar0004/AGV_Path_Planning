from navigation.route_manager import RouteManager


def main() -> None:
    """Demonstrate fixed-route execution with repeated tag frames."""
    route_manager = RouteManager()

    # Repeated tags simulate the same AprilTag staying visible for many frames.
    detected_tags = [
        0,
        0,
        0,
        1,
        1,
        2,
        2,
        2,
        3,
        3,
        3,
        4,
        4,
    ]

    for tag_id in detected_tags:
        action = route_manager.process_tag(tag_id)
        if action is not None:
            print(f"Route Action: {action.name}")


if __name__ == "__main__":
    main()
