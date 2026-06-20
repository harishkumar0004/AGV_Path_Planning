from agv_high_level.communication.esp32_client import ESP32Client
from agv_high_level.perception.tag_pose import TagPose


class TagPoseStreamer:
    def __init__(self, esp32: ESP32Client) -> None:
        self.esp32 = esp32
        self.was_visible = False

    def update(self, pose: TagPose) -> None:
        if pose.visible:
            self.esp32.send_tagpose(
                pose.tag_id,
                pose.x_m,
                pose.y_m,
                pose.yaw_deg,
            )
            self.was_visible = True
            return

        if self.was_visible:
            self.esp32.send_tag_lost()

        self.was_visible = False
