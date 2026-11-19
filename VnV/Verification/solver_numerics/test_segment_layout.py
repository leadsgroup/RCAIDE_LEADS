# VnV/Verification/solver_numerics/test_segment_layout.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import copy
import pickle

import numpy as np

from RCAIDE.Framework.Mission.Common import State, Conditions, Segment_Layout

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # Segment_Layout places the unknowns, residuals and conditions of a segment in one preallocated buffer.
    # The arrays in the state become views of that buffer, so writing to them writes to the buffer.
    # In these checks a leaf is one array in the state tree, and a slot is its place in the buffer.
    N = 4   # number of control points
    check_solver_slices_follow_pack_array_order(N)
    check_round_trip_preserves_values_and_memory_order(N)
    check_non_numeric_and_scalar_conditions_left_alone(N)
    check_aliases_share_one_slot(N)
    check_shared_memory_is_excluded(N)
    check_verify_catches_a_rebind_only(N)
    check_scalar_solver_variables_get_their_own_slots(N)
    check_boolean_solver_variable_rejected(N)
    check_relayout_of_a_converged_state_keeps_every_slot(N)
    check_residual_group_check(N)
    check_leaves_added_after_the_build_are_reported(N)
    check_unpack_array_writes_into_the_layout(N)
    check_rank_3_conditions_get_slots(N)
    check_assignment_writes_into_the_slot(N)
    check_assignment_that_cannot_reuse_the_slot_binds(N)
    check_slots_not_owned_keep_binding(N)
    check_unbound_conditions_unchanged(N)
    check_copies_share_the_layout_and_are_not_written_through(N)
    check_pickled_state_is_plain_data(N)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_solver_slices_follow_pack_array_order(N):
    # the solver slices hold the same order the old pack_array gave, unknowns then residuals then conditions
    state    = make_state(N)
    expected = {key: state.deep_get(key).pack_array() for key in
                ("unknowns.mission", "unknowns.network", "residuals.mission", "residuals.network")}
    layout   = Segment_Layout(state)
    for key, packed in expected.items():
        assert np.array_equal(layout.buffer[layout.slices[key]], packed)
        assert np.array_equal(state.deep_get(key).pack_array(), packed)
    unknowns = np.concatenate([expected["unknowns.mission"], expected["unknowns.network"]])
    assert np.array_equal(layout.buffer[layout.slices["unknowns"]], unknowns)
    assert layout.slices["unknowns"].stop == layout.slices["residuals"].start
    assert layout.slices["residuals"].stop == layout.slices["conditions"].start
    assert layout.slices["conditions"].stop == layout.buffer.size == 14 * N        # 6 N solver + 8 N conditions
    return


def check_round_trip_preserves_values_and_memory_order(N):
    # building the layout keeps every value and C or Fortran array order, and writes reach both the buffer and the leaves
    state  = make_state(N)
    before = {key: np.array(state.deep_get(key)) for key in
              ("unknowns.network.split", "conditions.freestream.density", "conditions.frames.rotations",
               "conditions.frames.force")}
    layout = Segment_Layout(state)
    for key, value in before.items():
        leaf = state.deep_get(key)
        assert leaf.shape == value.shape and leaf.dtype == np.float64
        assert np.array_equal(leaf, value, equal_nan=True)
        assert np.shares_memory(leaf, layout.buffer)
    assert state.conditions.frames.rotations.flags.f_contiguous
    assert state.conditions.frames.force.flags.c_contiguous

    # writes reach the buffer and the leaves, in both directions
    layout.buffer[layout.slices["unknowns.mission"]] = np.arange(2.0 * N)
    assert np.array_equal(state.unknowns.mission.throttle[:, 0], np.arange(N, 2.0 * N))
    state.conditions.freestream.mach[...] = 0.9
    assert np.all(layout.buffer[layout.slices["conditions"]][:N] == 0.9)
    return


def check_non_numeric_and_scalar_conditions_left_alone(N):
    # only float arrays move into the buffer: flags, integer indices and scalars stay as they are
    state  = make_state(N)
    flags, index = state.conditions.flags, state.conditions.index
    layout = Segment_Layout(state)
    assert state.conditions.flags is flags and state.conditions.index is index
    assert state.conditions.reference == 2.0
    assert ("conditions", "flags") not in layout.views
    return


def check_aliases_share_one_slot(N):
    # two names for one array take one slot, and stay the same array
    state = make_state(N)
    state.conditions.frames.alias = state.conditions.frames.force
    layout = Segment_Layout(state)
    assert state.conditions.frames.alias is state.conditions.frames.force
    assert np.shares_memory(state.conditions.frames.alias, layout.buffer)
    assert layout.buffer.size == 14 * N
    return


def check_shared_memory_is_excluded(N):
    # arrays that overlap other arrays or the previous segment stay outside the buffer
    state = make_state(N)
    whole = state.conditions.frames.force
    state.conditions.frames.force_x = whole[:, 0:1]                      # a view of another leaf
    state.conditions.freestream.mach = state.initials.conditions.freestream.mach  # held in initials too
    layout = Segment_Layout(state)
    assert state.conditions.frames.force is whole
    assert np.shares_memory(state.conditions.frames.force_x, whole)
    assert state.conditions.freestream.mach is state.initials.conditions.freestream.mach
    assert set(layout.excluded) == {("conditions", "frames", "force"), ("conditions", "frames", "force_x"),
                                    ("conditions", "freestream", "mach")}
    return


def check_verify_catches_a_rebind_only(N):
    # verify reports replaced or removed arrays, not values written in place
    state  = make_state(N)
    layout = Segment_Layout(state)
    assert layout.verify(state) == []
    state.conditions.freestream.mach[:] = 0.7                           # in place: still the view
    state.residuals.mission.force_x     = np.ones((N + 1, 1))             # a new shape rebinds to new memory
    del state.conditions.freestream["density"]                          # removed: no longer its view either
    assert layout.verify(state) == ["residuals.mission.force_x", "conditions.freestream.density"]
    return


def check_scalar_solver_variables_get_their_own_slots(N):
    # ground segments have scalar unknowns; each gets its own rank-0 slot
    state = make_state(N)
    state.unknowns.mission.elapsed_time       = 30          # ground segments; the same int object at both paths
    state.residuals.mission.final_velocity_error = 30
    expected = state.unknowns.mission.pack_array()
    layout   = Segment_Layout(state)
    assert state.unknowns.mission.elapsed_time.shape == ()
    assert state.unknowns.mission.elapsed_time is not state.residuals.mission.final_velocity_error
    assert np.array_equal(layout.buffer[layout.slices["unknowns.mission"]], expected)
    assert np.array_equal(state.unknowns.mission.pack_array(), expected)      # rank-0 views pack like numbers
    state.residuals.mission.final_velocity_error[...] = 1.5
    assert state.unknowns.mission.elapsed_time == 30 and layout.buffer[layout.slices["residuals.mission"]][-1] == 1.5
    return


def check_boolean_solver_variable_rejected(N):
    # a boolean cannot be solved for
    state = make_state(N)
    state.unknowns.mission.trim = True
    try:
        Segment_Layout(state)
    except TypeError:
        return
    raise AssertionError("Segment_Layout accepted a boolean solver variable")


def check_relayout_of_a_converged_state_keeps_every_slot(N):
    # after converge every leaf is a view of the old buffer; disjoint views of one owner do not overlap
    state  = make_state(N)
    first  = Segment_Layout(state)
    second = Segment_Layout(state)
    assert second.excluded == {} and second.buffer.size == first.buffer.size
    assert not np.shares_memory(state.conditions.freestream.mach, first.buffer)
    return


def check_residual_group_check(N):
    # verify can check one group only
    state  = make_state(N)
    layout = Segment_Layout(state)
    state.conditions.frames.force = np.ones((N, 4))
    assert layout.verify(state, group="residuals") == []
    assert layout.verify(state) == ["conditions.frames.force"]
    return


def check_leaves_added_after_the_build_are_reported(N):
    # added reports arrays created after the layout was built
    state  = make_state(N)
    layout = Segment_Layout(state)
    assert layout.added(state) == []
    state.conditions.freestream.speed_of_sound = np.ones((N, 1))
    assert layout.added(state) == ["conditions.freestream.speed_of_sound"]
    return


def check_unpack_array_writes_into_the_layout(N):
    # the network update unpacks its unknowns every iterate; the optimizer's writes must keep reaching them
    state  = make_state(N)
    layout = Segment_Layout(state)
    values = np.arange(2.0 * N) + 10.0
    state.unknowns.network.unpack_array(values)
    assert layout.verify(state) == []
    assert np.array_equal(layout.buffer[layout.slices["unknowns.network"]], values)
    return


def check_rank_3_conditions_get_slots(N):
    # rotation matrices are (N, 3, 3) and get slots as well
    state = make_state(N)
    state.conditions.frames.transform = np.arange(9.0 * N).reshape((N, 3, 3))
    value  = state.conditions.frames.transform.copy()
    layout = Segment_Layout(state)
    assert np.shares_memory(state.conditions.frames.transform, layout.buffer)
    assert np.array_equal(state.conditions.frames.transform, value)
    assert layout.buffer.size == 14 * N + 9 * N
    return


def check_assignment_writes_into_the_slot(N):
    # plain and item assignment of a same-shape float array write into the slot, as physics code writes them
    state  = make_state(N)
    layout = Segment_Layout(state)
    slot   = state.conditions.freestream.mach
    state.conditions.freestream.mach = np.full((N, 1), 0.7)              # plain assignment, as physics code writes it
    state.residuals.mission["force_x"] = np.full((N, 1), 2.0)            # item assignment too
    assert state.conditions.freestream.mach is slot and np.all(slot == 0.7)
    assert np.all(layout.buffer[layout.slices["residuals.mission"]][:N] == 2.0)
    assert layout.verify(state) == []
    return


def check_assignment_that_cannot_reuse_the_slot_binds(N):
    # another shape, a non-float array or a non-array replaces the leaf, and verify reports it
    state  = make_state(N)
    layout = Segment_Layout(state)
    state.conditions.freestream.mach = np.ones((N, 2))                   # new shape
    state.conditions.frames.force    = np.ones((N, 3), dtype=int)        # not float
    state.conditions.freestream.density = 1.0                            # not an array
    assert layout.verify(state) == ["conditions.freestream.mach", "conditions.freestream.density",
                                    "conditions.frames.force"]
    return


def check_slots_not_owned_keep_binding(N):
    # excluded leaves, aliased slots and views of a slot stored under another name all rebind, so the previous
    # segment, the other alias name and the viewed slot are never written through
    state = make_state(N)
    state.conditions.freestream.mach = state.initials.conditions.freestream.mach
    state.conditions.frames.alias    = state.conditions.frames.force
    layout  = Segment_Layout(state)
    initial = state.initials.conditions.freestream.mach
    state.conditions.freestream.mach = np.full((N, 1), 0.1)
    assert np.all(initial == 0.8)
    state.conditions.frames.alias = np.zeros((N, 3))
    assert not np.all(state.conditions.frames.force == 0.0)
    state.conditions.frames.column = state.conditions.frames.force[:, 0:1]
    state.conditions.frames.column = np.full((N, 1), -1.0)
    assert not np.any(state.conditions.frames.force == -1.0)
    return


def check_unbound_conditions_unchanged(N):
    # with no layout built, assignment replaces the array as usual
    state = make_state(N)
    mach  = state.conditions.freestream.mach
    state.conditions.freestream.mach = np.full((N, 1), 0.7)
    assert state.conditions.freestream.mach is not mach and np.all(mach == 0.85)
    return


def check_copies_share_the_layout_and_are_not_written_through(N):
    # a deepcopy keeps a reference to the layout, not its buffer, and writing to the copy leaves the state alone
    state  = make_state(N)
    layout = Segment_Layout(state)
    frames = copy.deepcopy(state.conditions.frames)
    assert object.__getattribute__(frames, "segment_layout") is layout          # not copied with its buffer
    frames.force = np.zeros((N, 3))
    assert not np.all(state.conditions.frames.force == 0.0) and layout.verify(state) == []
    return


def check_pickled_state_is_plain_data(N):
    # a pickled state does not carry the layout
    state = make_state(N)
    Segment_Layout(state)
    loaded = pickle.loads(pickle.dumps(state))
    assert object.__getattribute__(loaded.conditions.frames, "segment_layout") is None
    assert np.array_equal(loaded.conditions.frames.force, state.conditions.frames.force)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def make_state(N):
    """A small segment state: two solver groups (one with a 2-column leaf), conditions in C and F order, a NaN,
    non-float leaves, and initials from a previous segment."""
    state = State()
    state.unknowns.mission.pitch_angle     = np.linspace(0.0, 0.1, N)[:, None]
    state.unknowns.mission.throttle        = np.linspace(0.5, 0.8, N)[:, None]
    state.unknowns.network.split           = np.arange(2.0 * N).reshape((N, 2))
    state.residuals.mission.force_x        = np.linspace(-1.0, 1.0, N)[:, None]
    state.residuals.network.balance        = np.arange(N, dtype=float)[:, None]
    state.conditions.freestream            = Conditions()
    state.conditions.freestream.mach       = np.full((N, 1), 0.85)
    state.conditions.freestream.density    = np.array([[1.2], [np.nan], [0.4], [0.3]])
    state.conditions.frames                = Conditions()
    state.conditions.frames.rotations      = np.asfortranarray(np.arange(3.0 * N).reshape((N, 3)))
    state.conditions.frames.force          = np.arange(3.0 * N).reshape((N, 3))
    state.conditions.flags                 = np.zeros((N, 1), dtype=bool)
    state.conditions.index                 = np.arange(N)
    state.conditions.reference             = 2.0
    state.initials                         = State()
    state.initials.conditions.freestream   = Conditions()
    state.initials.conditions.freestream.mach = np.full((N, 1), 0.8)
    return state


if __name__ == '__main__':
    main()
