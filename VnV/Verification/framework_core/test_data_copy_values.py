# VnV/Verification/framework_core/test_data_copy_values.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import copy

import numpy as np

from RCAIDE.Framework.Core import Data

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # Data.copy_values copies another tree's values into this tree's existing arrays. Identical propulsors use it to
    # reuse results during a solve: the result must match a deepcopy without allocating new memory.
    check_values_match_a_deep_copy()
    check_existing_float_arrays_are_written_into()
    check_missing_or_mismatched_entries_get_independent_copies()
    check_keys_only_in_target_are_kept()
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_values_match_a_deep_copy():
    source = make_source()
    target = copy.deepcopy(source)
    target.outputs.thrust[...] = -1.0
    target.copy_values(source)
    reference = copy.deepcopy(source)
    for key in ("outputs.thrust", "outputs.vectors", "outputs.flags"):
        assert np.array_equal(target.deep_get(key), reference.deep_get(key))
    assert target.tag == "fan_0" and target.efficiency == 0.9
    return


def check_existing_float_arrays_are_written_into():
    source = make_source()
    target = copy.deepcopy(source)
    thrust = target.outputs.thrust
    target.copy_values(source)

    # same memory with new values, and still independent of the source
    assert target.outputs.thrust is thrust
    assert not np.shares_memory(target.outputs.thrust, source.outputs.thrust)
    return


def check_missing_or_mismatched_entries_get_independent_copies():
    # the target has thrust with another shape and no other outputs
    source = make_source()
    target = Data()
    target.outputs        = Data()
    target.outputs.thrust = np.zeros((5, 1))
    target.copy_values(source)
    assert target.outputs.thrust.shape == (4, 1)
    assert not np.shares_memory(target.outputs.thrust, source.outputs.thrust)
    assert not np.shares_memory(target.outputs.vectors, source.outputs.vectors)
    assert target.outputs.flags is not source.outputs.flags
    return


def check_keys_only_in_target_are_kept():
    source = make_source()
    target = copy.deepcopy(source)
    target.outputs.moment = np.ones((4, 3))
    target.copy_values(source)
    assert np.all(target.outputs.moment == 1.0)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def make_source():
    """A small results tree with float arrays, a boolean array, a string and a number."""
    source                  = Data()
    source.tag              = "fan_0"
    source.outputs          = Data()
    source.outputs.thrust   = np.arange(4.0)[:, None]
    source.outputs.vectors  = np.arange(12.0).reshape((4, 3))
    source.outputs.flags    = np.array([True, False])
    source.efficiency       = 0.9
    return source


if __name__ == '__main__':
    main()
