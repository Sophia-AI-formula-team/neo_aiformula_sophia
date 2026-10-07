"""Inference-only A3 shared MobileNet/FPN entrypoint."""
from .shared_runtime import SharedFPNPerception
from .contracts import ROS_TOPICS
from .outputs import Detection, PerceptionOutput

PerceptionModel = SharedFPNPerception

__all__ = ["PerceptionModel", "SharedFPNPerception", "Detection", "PerceptionOutput", "ROS_TOPICS"]
