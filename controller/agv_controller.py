from typing import Any

import cv2
import numpy as np

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
        self.window_name = "AGV Controller"

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
        detections = self.perception_manager.update()

        generated_events = self.event_generator.generate_events()
        if not generated_events:
            print(f"Current State: {self.state_machine.current_state.name}")
        else:
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

        self._display_camera_frame(detections)

    def shutdown(self) -> None:
        """Release controller resources safely."""
        self.perception_manager.release()
        cv2.destroyAllWindows()
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

    def _display_camera_frame(self, detections: list[dict[str, Any]]) -> None:
        """
        Display the latest camera frame with simple debugging overlays.

        Args:
            detections: AprilTag detections from the current frame.
        """
        frame = self.perception_manager.last_frame
        if frame is None:
            return

        display_frame = frame.copy()
        self._draw_detections(display_frame, detections)
        self._draw_status_overlay(display_frame)

        cv2.imshow(self.window_name, display_frame)

        # Press q to close the window, release the camera, and stop the loop.
        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("Closing AGVController camera window.")
            self.shutdown()

    def _draw_detections(
        self,
        frame: np.ndarray,
        detections: list[dict[str, Any]],
    ) -> None:
        """
        Draw AprilTag outlines and IDs on the camera frame.

        Args:
            frame: Camera image being displayed.
            detections: AprilTag detections from the current frame.
        """
        for detection in detections:
            tag_id = detection["tag_id"]
            center_x = int(detection["center_x"])
            center_y = int(detection["center_y"])
            corners = np.array(detection["corners"], dtype=np.int32)

            cv2.polylines(
                frame,
                [corners],
                isClosed=True,
                color=(0, 255, 0),
                thickness=2,
            )
            cv2.circle(
                frame,
                (center_x, center_y),
                radius=5,
                color=(0, 0, 255),
                thickness=-1,
            )
            cv2.putText(
                frame,
                f"Tag ID: {tag_id}",
                (center_x + 10, center_y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

    def _draw_status_overlay(self, frame: np.ndarray) -> None:
        """
        Draw current AGV state and latest tag ID on the camera frame.

        Args:
            frame: Camera image being displayed.
        """
        perception = self.application_state.perception
        tag_text = (
            f"Tag ID: {perception.tag_id}"
            if perception.tag_visible
            else "Tag ID: None"
        )

        cv2.putText(
            frame,
            f"State: {self.state_machine.current_state.name}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            tag_text,
            (20, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )
