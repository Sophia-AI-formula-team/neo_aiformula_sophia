"""Recheck the original nominal controls and immediate stop against new cone feet."""
from dataclasses import replace

import numpy as np

from concept_candidates.belief61515 import braking_branch
from concept_candidates.curve_profile import evaluate_profile
from concept_road.v3_model import BODY_FRONT, BODY_HALF_WIDTH, BODY_REAR


def launch_clearance(objects, obs, pending, audit, ids):
    # Certified launches already have a sustained nominal path. Restore the
    # identical nominal 50 ms controls from the planner's deduplicated batch,
    # including changing drive/steer on the field edge candidate.
    groups, restore = np.unique(pending['nominal_indices'][ids], return_inverse=True)
    sequence = np.asarray(pending['nominal_substeps'])[:, groups, :].transpose(1, 0, 2)
    scene = replace(obs, objects=objects)
    nominal = evaluate_profile(scene, sequence, audit['horizon'], 0.)
    first_drive, first_steer = sequence[:, 0, :].T
    stopped, bound, brake_path, _ = braking_branch(scene, first_drive, first_steer, 0., details=True)
    # The statistical test alone could assign little probability to a very
    # spread-out target directly on a path. Also require clearance to each
    # measured centre for both the nominal trajectory and the brake branch.
    centres = np.vstack([np.zeros((len(groups), 1, 2)).transpose(1, 0, 2), brake_path.transpose(1, 0, 2)])
    body_radius = np.hypot(max(BODY_FRONT, BODY_REAR), BODY_HALF_WIDTH)
    geometric = nominal['geometric'].copy()
    for cone in objects[::2]:
        distances = np.linalg.norm(centres-cone[:2], axis=2)
        geometric = np.minimum(geometric, distances.min(axis=0)-body_radius-cone[4]-.07)
    margin = np.minimum.reduce([nominal['obstacle_lower'], bound, geometric])
    return np.where(stopped & np.isfinite(margin), margin, -np.inf)[restore]
