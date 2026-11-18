"""Runtime source extracted from the accepted V6.26 freeze: experiments/v615_recovery/compare61513.py. """

from dataclasses import dataclass

import numpy as np

from concept_road.v34_road import RoadObservation

@dataclass(frozen=True)
class Guided(RoadObservation):
    guidance:np.ndarray=None
