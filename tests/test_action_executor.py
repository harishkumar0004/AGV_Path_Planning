from control.action_executor import ActionExecutor
from state_machine.states import AGVState


def main() -> None:
    """Demonstrate state-to-motor-action mappings."""
    action_executor = ActionExecutor()

    # These calls only print simulated commands. No hardware is used.
    action_executor.execute(AGVState.SEARCHING)
    action_executor.execute(AGVState.ALIGNING)
    action_executor.execute(AGVState.APPROACHING)
    action_executor.execute(AGVState.STOPPED)


if __name__ == "__main__":
    main()
