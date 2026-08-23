# hp_decomposition_test.py
# Created:  Aug 2026, M. Clarke

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Mission.Solver.hp_decomposition import (
    compute_subsegment_layout,
    hp_decompose_segment,
    EXTENT_ATTRIBUTES,
)

import numpy as np


def main():
    test_compute_subsegment_layout()
    test_hp_decompose_segment_linear_pair()
    test_hp_decompose_segment_defaults_from_numerics()
    test_hp_decompose_segment_divide()
    test_hp_decompose_segment_battery_discharge()
    test_hp_decompose_segment_dual_spec()
    test_hp_decompose_segment_cumulative()
    test_hp_decompose_segment_no_split_needed()
    test_hp_decompose_segment_unregistered_type_raises()


# ----------------------------------------------------------------------
#   compute_subsegment_layout
# ----------------------------------------------------------------------
def test_compute_subsegment_layout():

    # worked examples from the A.4 design discussion: N=16 points, floor=4.
    # Smallest, not largest: at U=1, three divisors (4,8,16) all clear the
    # dimension cap -- the old max-based algorithm would have picked 16 (no
    # split at all); the calibration-driven min-based algorithm picks 4.
    assert compute_subsegment_layout(16, 1, max_dimension=100, min_control_points=4) == (4, 4)

    # the actual departure_transition_1 case: 16 points, 5 unknowns/point
    # (2 mission: throttle, thrust_vector_angle; 3 network: motor_current,
    # cell_temperature, cell_state_of_charge). Only n=4 clears the dimension
    # cap here (n=8 -> dim 40 >= 32), so floor and dimension cap agree.
    assert compute_subsegment_layout(16, 5, max_dimension=32, min_control_points=4) == (4, 4)

    # floor equal to N: the only divisor >= floor is N itself, so no split
    # happens regardless of how loose the dimension cap is
    assert compute_subsegment_layout(8, 2, max_dimension=32, min_control_points=8) == (8, 1)

    # min_control_points floor takes precedence over a smaller dimension-
    # satisfying divisor when both exist
    n, k = compute_subsegment_layout(16, 3, max_dimension=100, min_control_points=8)
    assert n >= 8
    assert 16 % n == 0

    # no valid decomposition: unknowns alone (40) already exceed max_dimension (32)
    # even at the smallest possible n=1 -- must raise, not silently emit n=1
    raised = False
    try:
        compute_subsegment_layout(16, 40, max_dimension=32, min_control_points=4)
    except ValueError:
        raised = True
    assert raised, "expected ValueError when unknowns alone exceed max_dimension"

    # no valid decomposition: floor and dimension cap conflict (every divisor
    # of N that clears the floor blows the dimension cap)
    raised = False
    try:
        compute_subsegment_layout(16, 6, max_dimension=20, min_control_points=4)
    except ValueError:
        raised = True
    assert raised, "expected ValueError when the floor and dimension cap can't both be satisfied"

    print("test_compute_subsegment_layout: PASS")


# ----------------------------------------------------------------------
#   hp_decompose_segment -- ('pair', start, end) extent
# ----------------------------------------------------------------------
def test_hp_decompose_segment_linear_pair():
    Segments = RCAIDE.Framework.Mission.Segments
    base = Segments.Segment()
    segment = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base)
    segment.tag              = "departure_transition_1"
    segment.air_speed_start  = 15 * Units.mph
    segment.air_speed_end    = 35 * Units.mph
    segment.acceleration     = 0.2
    segment.state.numerics.number_of_control_points = 16

    pieces = hp_decompose_segment(segment, number_of_unknowns=5, tolerance=1e-4, step_size=1e-5,
                                   max_dimension=32, min_control_points=4)

    assert len(pieces) == 4
    for piece in pieces:
        assert piece.state.numerics.number_of_control_points == 4
        assert piece.state.numerics.mission_solver.tolerance == 1e-4
        assert piece.state.numerics.mission_solver.step_size == 1e-5

    # boundary values chained correctly: piece i's end == piece i+1's start,
    # first piece starts at the original segment's start, last piece ends
    # at the original segment's end
    expected_edges = np.linspace(15 * Units.mph, 35 * Units.mph, 5)
    for i, piece in enumerate(pieces):
        assert abs(piece.air_speed_start - expected_edges[i]) < 1e-9
        assert abs(piece.air_speed_end - expected_edges[i + 1]) < 1e-9
    assert abs(pieces[0].air_speed_start - segment.air_speed_start) < 1e-9
    assert abs(pieces[-1].air_speed_end - segment.air_speed_end) < 1e-9

    # tags are distinct and traceable back to the parent
    assert len({p.tag for p in pieces}) == 4
    for p in pieces:
        assert p.tag.startswith("departure_transition_1_")

    print("test_hp_decompose_segment_linear_pair: PASS")


# ----------------------------------------------------------------------
#   hp_decompose_segment -- max_dimension/min_control_points default from
#   Numerics.py's hp_decomposition block when not passed explicitly
# ----------------------------------------------------------------------
def test_hp_decompose_segment_defaults_from_numerics():
    Segments = RCAIDE.Framework.Mission.Segments
    base = Segments.Segment()
    segment = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base)
    segment.tag              = "departure_transition_1"
    segment.air_speed_start  = 15 * Units.mph
    segment.air_speed_end    = 35 * Units.mph
    segment.acceleration     = 0.2
    segment.state.numerics.number_of_control_points = 16

    default_max_dim = segment.state.numerics.hp_decomposition.max_dimension
    default_floor    = segment.state.numerics.hp_decomposition.min_control_points
    assert default_max_dim == 64
    assert default_floor == 4

    # no max_dimension/min_control_points passed -- must fall back to
    # segment.state.numerics.hp_decomposition, not silently use something else.
    pieces = hp_decompose_segment(segment, number_of_unknowns=2, tolerance=1e-4, step_size=1e-5)
    n, k = compute_subsegment_layout(16, 2, max_dimension=default_max_dim, min_control_points=default_floor)
    assert len(pieces) == k
    for piece in pieces:
        assert piece.state.numerics.number_of_control_points == n

    # overriding the segment's own numerics changes the outcome, confirming the
    # fallback actually reads from the segment, not a hardcoded module-level
    # constant. Under min-based selection, tightening the cap only changes the
    # chosen n once it excludes the smallest candidate entirely (n=4 -> dim 8
    # stays valid as long as max_dimension > 8, since min-selection never
    # prefers a larger n just because the cap loosened) -- so tighten past
    # that point and confirm it now raises instead of silently keeping n=4.
    segment.state.numerics.hp_decomposition.max_dimension = 6
    raised = False
    try:
        hp_decompose_segment(segment, number_of_unknowns=2, tolerance=1e-4, step_size=1e-5)
    except ValueError:
        raised = True
    assert raised, "tightened max_dimension must actually be read from the segment, not ignored"

    print("test_hp_decompose_segment_defaults_from_numerics: PASS")


# ----------------------------------------------------------------------
#   hp_decompose_segment -- ('divide', attr) extent
# ----------------------------------------------------------------------
def test_hp_decompose_segment_divide():
    Segments = RCAIDE.Framework.Mission.Segments
    base = Segments.Segment()
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base)
    segment.tag       = "cruise"
    segment.air_speed = 110 * Units["mph"]
    segment.altitude  = 1000 * Units.ft
    segment.distance  = 40 * Units.nmi
    segment.state.numerics.number_of_control_points = 16

    pieces = hp_decompose_segment(segment, number_of_unknowns=5, tolerance=1e-4, step_size=1e-5,
                                   max_dimension=32, min_control_points=4)

    assert len(pieces) == 4
    for piece in pieces:
        assert abs(piece.distance - segment.distance / 4) < 1e-9
        # air_speed/altitude untouched -- not part of this class's extent spec
        assert piece.air_speed == segment.air_speed
        assert piece.altitude == segment.altitude

    print("test_hp_decompose_segment_divide: PASS")


# ----------------------------------------------------------------------
#   hp_decompose_segment -- Ground.Battery_Discharge, a second ('divide', ...)
#   type on a structurally different segment (Ground, not Cruise/Climb)
# ----------------------------------------------------------------------
def test_hp_decompose_segment_battery_discharge():
    Segments = RCAIDE.Framework.Mission.Segments
    base = Segments.Segment()
    segment = Segments.Ground.Battery_Discharge(base)
    segment.tag  = "discharge"
    segment.time = 300 * Units.seconds
    segment.state.numerics.number_of_control_points = 16

    pieces = hp_decompose_segment(segment, number_of_unknowns=1, tolerance=1e-4, step_size=1e-5,
                                   max_dimension=8, min_control_points=4)

    n, k = compute_subsegment_layout(16, 1, max_dimension=8, min_control_points=4)
    assert len(pieces) == k
    for piece in pieces:
        assert abs(piece.time - segment.time / k) < 1e-9

    print("test_hp_decompose_segment_battery_discharge: PASS")


# ----------------------------------------------------------------------
#   hp_decompose_segment -- two specs applied together (lockstep)
# ----------------------------------------------------------------------
def test_hp_decompose_segment_dual_spec():
    Segments = RCAIDE.Framework.Mission.Segments
    base = Segments.Segment()
    segment = Segments.Climb.Constant_Mach_Linear_Altitude(base)
    segment.tag             = "climb"
    segment.altitude_start  = 1000 * Units.ft
    segment.altitude_end    = 5000 * Units.ft
    segment.distance        = 20 * Units.nmi
    segment.state.numerics.number_of_control_points = 12

    pieces = hp_decompose_segment(segment, number_of_unknowns=4, tolerance=1e-4, step_size=1e-5,
                                   max_dimension=16, min_control_points=3)

    n, k = compute_subsegment_layout(12, 4, max_dimension=16, min_control_points=3)
    assert len(pieces) == k

    # both specs must move together: altitude and distance each split into k
    # equal pieces, independently confirmable, but registered as one class
    # precisely so a future re-split can't move one without the other
    total_distance = sum(p.distance for p in pieces)
    assert abs(total_distance - segment.distance) < 1e-6
    assert abs(pieces[0].altitude_start - segment.altitude_start) < 1e-9
    assert abs(pieces[-1].altitude_end - segment.altitude_end) < 1e-9

    print("test_hp_decompose_segment_dual_spec: PASS")


# ----------------------------------------------------------------------
#   hp_decompose_segment -- ('cumulative', target, source) extent:
#   heading continuity for a curved segment
# ----------------------------------------------------------------------
def test_hp_decompose_segment_cumulative():
    Segments = RCAIDE.Framework.Mission.Segments
    base = Segments.Segment()
    segment = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base)
    segment.tag         = "turn"
    segment.air_speed   = 110 * Units["mph"]
    segment.altitude    = 1000 * Units.ft
    segment.turn_radius = 3600 * Units.ft
    segment.turn_angle  = 90 * Units.degree
    segment.true_course = 30 * Units.degree  # starts mid-turn from a prior leg, not due north
    segment.state.numerics.number_of_control_points = 16

    n, k = compute_subsegment_layout(16, 2, max_dimension=32, min_control_points=4)
    pieces = hp_decompose_segment(segment, number_of_unknowns=2, tolerance=1e-4, step_size=1e-5,
                                   max_dimension=32, min_control_points=4)
    assert len(pieces) == k

    # each piece turns 1/k of the total arc
    for piece in pieces:
        assert abs(piece.turn_angle - segment.turn_angle / k) < 1e-9

    # true_course accumulates: piece 0 starts at the original heading, each
    # later piece starts where the previous one's turn would have ended --
    # this is the actual bug being fixed (true_course isn't chained via
    # state.initials the way position/velocity/time are)
    expected_starts = segment.true_course + np.arange(k) * (segment.turn_angle / k)
    for i, piece in enumerate(pieces):
        assert abs(piece.true_course - expected_starts[i]) < 1e-9

    # the last piece's start + its own turn reaches the original total heading change
    assert abs((pieces[-1].true_course + pieces[-1].turn_angle) - (segment.true_course + segment.turn_angle)) < 1e-9

    print("test_hp_decompose_segment_cumulative: PASS")


# ----------------------------------------------------------------------
#   hp_decompose_segment -- already small enough, no-op
# ----------------------------------------------------------------------
def test_hp_decompose_segment_no_split_needed():
    Segments = RCAIDE.Framework.Mission.Segments
    base = Segments.Segment()
    segment = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base)
    segment.tag             = "short_segment"
    segment.air_speed_start = 15 * Units.mph
    segment.air_speed_end   = 20 * Units.mph
    segment.acceleration    = 0.2
    segment.state.numerics.number_of_control_points = 4

    pieces = hp_decompose_segment(segment, number_of_unknowns=2, tolerance=1e-6, step_size=1e-8,
                                   max_dimension=32, min_control_points=4)

    # 4*2=8 < 32 already -- layout should pick n=4, k=1: the original segment
    # object, unmodified, not a copy
    assert len(pieces) == 1
    assert pieces[0] is segment

    print("test_hp_decompose_segment_no_split_needed: PASS")


# ----------------------------------------------------------------------
#   hp_decompose_segment -- unregistered segment type
# ----------------------------------------------------------------------
def test_hp_decompose_segment_unregistered_type_raises():
    Segments = RCAIDE.Framework.Mission.Segments
    base = Segments.Segment()
    # Takeoff is deliberately unregistered: velocity profile and total
    # elapsed time are both solved unknowns there, not prescribed, plus a
    # standing reputation for finicky convergence independent of this
    # feature -- not worth registering without real motivation. Confirm it
    # still fails loudly rather than silently mis-splitting. Single_Point
    # isn't a useful check here: it's hardcoded to 1 control point, so
    # layout always picks number_of_subsegments=1 and returns before ever
    # reaching the registry.
    segment = Segments.Ground.Takeoff(base)
    segment.tag = "takeoff"
    segment.state.numerics.number_of_control_points = 16
    assert type(segment) not in EXTENT_ATTRIBUTES

    raised = False
    try:
        hp_decompose_segment(segment, number_of_unknowns=2, tolerance=1e-6, step_size=1e-8,
                              max_dimension=8, min_control_points=1)
    except ValueError:
        raised = True
    assert raised, "expected ValueError for an unregistered segment type"

    print("test_hp_decompose_segment_unregistered_type_raises: PASS")


if __name__ == '__main__':
    main()
