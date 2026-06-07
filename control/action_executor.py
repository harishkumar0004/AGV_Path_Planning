from control.motor_controller import MotorController
from state_machine.states import AGVState


class ActionExecutor:
    """Converts AGV states into simulated motor controller actions."""

    def __init__(self) -> None:
        """Create an action executor with its own motor controller."""
        self.motor_controller = MotorController()
        self.search_speed = 25
        self.align_speed = 25
        self.approach_speed = 50

    def execute(self, state: AGVState) -> None:
        """
        Execute the motor action mapped to the current AGV state.

        Args:
            state: Current high-level AGV state.
        """
        print(f"State: {state.name}")

        if state == AGVState.SEARCHING:
            print("Action: TURN_LEFT")
            self.motor_controller.turn_left(self.search_speed)
            return

        if state == AGVState.ALIGNING:
            print("Action: TURN_LEFT")
            self.motor_controller.turn_left(self.align_speed)
            return

        if state == AGVState.APPROACHING:
            print("Action: FORWARD")
            self.motor_controller.forward(self.approach_speed)
            return

        if state == AGVState.STOPPED:
            print("Action: STOP")
            self.motor_controller.stop()
            return

        # Any unmapped state should be safe in this simulation layer.
        print("Action: STOP")
        self.motor_controller.stop()
