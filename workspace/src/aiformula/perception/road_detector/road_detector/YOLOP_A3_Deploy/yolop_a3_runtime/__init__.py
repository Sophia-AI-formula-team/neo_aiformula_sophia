"""A4 inference entrypoint; retain the published module name for existing callers."""
from .shared_runtime import TemporalPerceptionModel
from .contracts import ROS_TOPICS
from .outputs import Detection, PerceptionOutput

RELEASE_NAME = "A4"
__version__ = "4.0.0"

PerceptionModel = TemporalPerceptionModel
# Retain the published Python name for existing importers; no second model.
SharedFPNPerception = TemporalPerceptionModel

__all__ = ["PerceptionModel", "TemporalPerceptionModel", "SharedFPNPerception", "Detection", "PerceptionOutput", "ROS_TOPICS"]
