"""Runtime source extracted from the accepted V6.26 freeze: experiments/v617_regression/rear_reference617.py. """

from dataclasses import dataclass

from concept_road.rear_road import RecoveryObservation

@dataclass(frozen=True)
class RecoveryReference(RecoveryObservation):
    reference: object = None
