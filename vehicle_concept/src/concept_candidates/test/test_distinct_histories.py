import numpy as np
import pytest


def test_only_bit_identical_histories_share_checks_and_slots_restore_in_original_order():
    batch = pytest.importorskip('concept_candidates.trajectory_batch')
    state = np.zeros((6,5,8))
    state[2,1,0] = np.nextafter(0.,1.)
    state[3,3,6] = .01
    enabled = np.ones((6,5),bool)
    enabled[4,4] = False
    take,restore = batch.distinct_histories(state,(0,1,2,5,6),enabled)
    assert take == [0,1,3,4]
    np.testing.assert_array_equal(restore,[0,1,0,2,3])
