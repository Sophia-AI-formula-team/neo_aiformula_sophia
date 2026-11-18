import numpy as np

from concept_candidates.candidate_merge import distinct_requests


def test_stable_original_ids_and_no_approximate_merging():
    low, higher = .5, np.nextafter(.5, 1.)
    targets = [1., 1., 1.001, 1., 0., -0., 1., 1.]
    speeds = [low, low, low, higher, low, low, 0., .001]
    representatives, inverse = distinct_requests(targets, speeds)
    assert representatives.tolist() == [0, 2, 3, 4, 5, 6, 7]
    assert representatives[inverse].tolist() == [0, 0, 2, 3, 4, 5, 6, 7]


def test_empty_and_already_distinct_batches():
    reps, inverse = distinct_requests([], [])
    assert reps.size == inverse.size == 0
    reps, inverse = distinct_requests([1., 0., -1.], [0., 0., 0.])
    assert reps.tolist() == inverse.tolist() == [0, 1, 2]
