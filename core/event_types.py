from enum import Enum, auto


class PerceptionEvent(Enum):
    """Events generated from changes in perception state."""

    # Generated when no tag was visible before, and a tag is visible now.
    TAG_DETECTED = auto()

    # Generated when a tag was visible before, and no tag is visible now.
    TAG_LOST = auto()

    # Generated when the visible tag ID changes.
    TAG_CHANGED = auto()
