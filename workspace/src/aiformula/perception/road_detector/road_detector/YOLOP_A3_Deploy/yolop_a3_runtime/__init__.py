"""Inference-only A3 V76 shared MobileNet/FCOS entrypoint."""
from .shared_runtime import TemporalPerceptionModel
from .contracts import ROS_TOPICS
from .outputs import Detection, PerceptionOutput

PerceptionModel = TemporalPerceptionModel
# Retain the published Python name for existing importers; no second model.
SharedFPNPerception = TemporalPerceptionModel

__all__ = ["PerceptionModel", "TemporalPerceptionModel", "SharedFPNPerception", "Detection", "PerceptionOutput", "ROS_TOPICS"]
