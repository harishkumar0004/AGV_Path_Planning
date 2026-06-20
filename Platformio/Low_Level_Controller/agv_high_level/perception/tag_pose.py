from dataclasses import dataclass
import time


@dataclass(frozen=True)
class TagPose:
    visible: bool
    tag_id: int
    x_m: float
    y_m: float
    z_m: float
    yaw_deg: float
    timestamp: float

    @staticmethod
    def lost() -> "TagPose":
        return TagPose(
            visible=False,
            tag_id=-1,
            x_m=0.0,
            y_m=0.0,
            z_m=0.0,
            yaw_deg=0.0,
            timestamp=time.monotonic(),
        )
