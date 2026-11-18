"""Runtime source extracted from the accepted V6.26 freeze: research/learned_racing/unity_laps/v34_road.py. """

from dataclasses import dataclass

import numpy as np

from concept_road.v3_model import Observation

@dataclass(frozen=True)
class RoadObservation(Observation):
    heading: np.ndarray = None
