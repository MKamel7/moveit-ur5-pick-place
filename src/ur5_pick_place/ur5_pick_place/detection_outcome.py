"""Classify what perception actually did, with no ROS in it.

WHY A SEPARATE MODULE. The cell inferred perception's state from topic
availability: `get_perceived_top` subscribes to `/detected/<colour>`, spins
until a message arrives or a deadline passes, and returns the pose or None.
None is then read as "no object". It is not. It is the union of at least four
different conditions:

  - perception ran and the scene is empty                 (a real observation)
  - the detector is not running, or crashed               (a broken pipeline)
  - nobody ever launched it                               (a broken bringup)
  - a pose arrived, latched, minutes old, still retained  (a stale answer)

The last one is the dangerous member, because it does not look like a failure
at all. `/detected/<colour>` is TRANSIENT_LOCAL, so a subscriber joining later
is immediately handed the last pose that was ever published, whether or not
anything is producing them now. That is exactly what happened when the Gazebo
server had died: the topic still answered instantly, with the part's home pose
from before the run, and the placement benchmark scored three trials against it
and reported a 126 mm perception error with a straight face.

So the classifier below asks for the pose AND its age AND whether a publisher
exists, and refuses to collapse them into one boolean. Age is compared to a
budget rather than trusted, because a latched pose is always available and
almost never current.
"""

from __future__ import annotations

from dataclasses import dataclass

# Result codes, mirroring DetectObject.action so the two cannot drift silently.
NONE = 0
NO_DETECTION = 1
PERCEPTION_UNAVAILABLE = 2
STALE = 3
INVALID_COLOR = 4

VALID_COLORS = ("red", "green", "blue")

# How old a pose may be and still describe the scene. The detector publishes at
# camera rate, so anything approaching a second means the pipeline stopped
# rather than that the object moved slowly.
DEFAULT_MAX_AGE_S = 1.0


@dataclass(frozen=True)
class DetectionOutcome:
    success: bool
    failure: int
    message: str


def classify(
    color: str,
    pose_received: bool,
    age_s: float | None,
    publisher_count: int,
    max_age_s: float = DEFAULT_MAX_AGE_S,
) -> DetectionOutcome:
    """Decide what a detection attempt means.

    Order matters. An unknown colour is a caller bug and is reported before
    anything is measured. A missing publisher outranks a missing pose, because
    "nothing is producing detections" is a truer statement than "nothing was
    detected" and sends whoever reads it to the right place. Staleness is
    checked last and only when a pose actually arrived.
    """
    if color not in VALID_COLORS:
        return DetectionOutcome(False, INVALID_COLOR,
                                f"unknown colour {color!r}, expected one of {VALID_COLORS}")

    if publisher_count == 0:
        # Reported even when a latched pose is in hand: a retained sample from a
        # dead publisher is the failure this whole module exists for.
        return DetectionOutcome(False, PERCEPTION_UNAVAILABLE,
                                f"nothing is publishing detections for {color}")

    if not pose_received:
        return DetectionOutcome(False, NO_DETECTION,
                                f"perception is running and reports no {color} object")

    if age_s is None:
        # A pose with no usable stamp cannot be shown to be current, and
        # assuming it is current is the assumption that failed before.
        return DetectionOutcome(False, STALE,
                                "pose carries no timestamp, so its age cannot be checked")

    if age_s > max_age_s:
        return DetectionOutcome(False, STALE,
                                f"pose is {age_s:.2f}s old, older than the {max_age_s:.2f}s budget")

    return DetectionOutcome(True, NONE, "")
