# RCAIDE/Library/Mission/Solver/hp_decomposition.py
#
#
# Created:  Aug 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_subsegment_layout
# ----------------------------------------------------------------------------------------------------------------------
def compute_subsegment_layout(number_of_control_points, number_of_unknowns, max_dimension=32, min_control_points=4):
    """Static (design-time) hp-decomposition layout: how many sub-segments a
    segment should be split into, and how many control points each gets.

    Finds the largest divisor of number_of_control_points that (a) keeps
    total sub-problem dimension (points-per-piece * number_of_unknowns)
    under max_dimension and (b) stays at or above min_control_points.
    Largest, not smallest, because fewer sub-segments means less chained-
    segment overhead while still respecting the dimension cap. Requiring an
    exact divisor keeps every sub-segment the same size -- no ragged last
    piece with fewer points than the rest.

    This is a pure function of static problem size (never of solver
    behavior/convergence outcome), so it returns the same layout every time
    for the same segment definition -- required for later JIT/AD reuse
    (Track B), where changing array shapes at runtime based on how a solve
    went would defeat trace reuse entirely.

    Assumptions:
    max_dimension=32 and min_control_points=4 are placeholders pending a
    proper calibration sweep (see RCAIDE_compute_acceleration_path.md A.4)
    -- only two real data points exist so far (24 dims converged in
    seconds, 96 dims did not converge in 15+ minutes on the same segment).

    Source:
    N/A

    Inputs:
    number_of_control_points  [int]  original segment's control-point count
    number_of_unknowns        [int]  mission + network unknowns per control
                                     point (network unknowns included if the
                                     segment's network_solver.type is None,
                                     i.e. they're folded into the same
                                     problem the mission solver sees)
    max_dimension             [int]  upper bound on points_per_piece * number_of_unknowns
    min_control_points        [int]  floor on points per sub-segment

    Outputs:
    (points_per_subsegment, number_of_subsegments) [tuple of int]

    Properties Used:
    N/A
    """
    if number_of_control_points < 1 or number_of_unknowns < 1:
        raise ValueError("number_of_control_points and number_of_unknowns must be >= 1")

    divisors   = [n for n in range(1, number_of_control_points + 1) if number_of_control_points % n == 0]
    candidates = [n for n in divisors if n >= min_control_points and n * number_of_unknowns < max_dimension]

    if not candidates:
        raise ValueError(
            f"No valid hp-decomposition: no divisor of {number_of_control_points} control points "
            f"keeps sub-problem dimension < {max_dimension} (unknowns={number_of_unknowns}) while "
            f"staying >= the {min_control_points}-point floor. This means too many simultaneously-"
            f"active control variables, not a control-point problem -- reduce active control "
            f"variables, raise max_dimension, or lower min_control_points."
        )

    points_per_subsegment = max(candidates)
    number_of_subsegments = number_of_control_points // points_per_subsegment
    return points_per_subsegment, number_of_subsegments


# ----------------------------------------------------------------------------------------------------------------------
#  extent-attribute registry
# ----------------------------------------------------------------------------------------------------------------------
# Segment classes that hp_decompose_segment knows how to split. Each entry is
# either a (start_attr, end_attr) pair -- linearly interpolated across the
# sub-segments -- or a single attr name representing a total extent that gets
# divided evenly. Only segment types actually verified against their own
# initialize_conditions code belong here (see RCAIDE_compute_acceleration_path.md
# A.4's per-segment-type audit for the rest -- most are safe by inspection but
# unverified in this pass).
LINEAR_PAIR_EXTENT_ATTRIBUTES = {}
DIVIDE_EXTENT_ATTRIBUTES      = {}


def _register_linear_pair(segment_class, start_attr, end_attr):
    LINEAR_PAIR_EXTENT_ATTRIBUTES[segment_class] = (start_attr, end_attr)


def _register_divide(segment_class, attr):
    DIVIDE_EXTENT_ATTRIBUTES[segment_class] = attr


def _register_known_segment_types():
    # Imported lazily to avoid a hard import-order dependency between this
    # module and RCAIDE.Framework.Mission.Segments at package-init time.
    from RCAIDE.Framework.Mission.Segments import Cruise, Climb

    _register_linear_pair(Cruise.Constant_Acceleration_Constant_Altitude, 'air_speed_start', 'air_speed_end')
    _register_divide(Cruise.Constant_Speed_Constant_Altitude, 'distance')
    _register_linear_pair(Climb.Constant_Throttle_Constant_Speed, 'altitude_start', 'altitude_end')


_register_known_segment_types()


# ----------------------------------------------------------------------------------------------------------------------
#  hp_decompose_segment
# ----------------------------------------------------------------------------------------------------------------------
def hp_decompose_segment(segment, number_of_unknowns, tolerance, step_size,
                          max_dimension=32, min_control_points=4):
    """Splits one segment into a chain of smaller sub-segments of the same
    type, covering the same overall extent, per compute_subsegment_layout's
    static sizing rule.

    Reuses RCAIDE's existing segment-chaining (state.initials) for
    continuity between pieces -- this is not new solver machinery, just
    more of what already chains different-typed segments in a mission
    together.

    tolerance/step_size are required, not defaulted, on purpose: neither
    the parent segment's own settings nor Numerics.py's library default
    turned out to be a safe stand-in. Copying the parent segment's settings
    unchanged (tuned for its original, larger/more expensive dimension --
    e.g. departure_transition_1's 1E-2/1E-3) left real accuracy on the
    table on a much smaller, now-cheap sub-problem (2.7% final-SOC
    discrepancy vs. a monolithic reference). Numerics.py's own library
    default (1E-6/1E-8) did better but still not well -- 1.9% discrepancy,
    worse than a since-tuned 1E-4/1E-5 (0.3%) -- because the right FD step
    size depends on this vehicle's own physics-model noise floor (rotor/
    aero surrogates), not on problem size alone. Pick deliberately per
    vehicle rather than trust either inherited value; see A.4 in
    RCAIDE_compute_acceleration_path.md for the calibration data so far.

    Assumptions:
    segment's class is registered in LINEAR_PAIR_EXTENT_ATTRIBUTES or
    DIVIDE_EXTENT_ATTRIBUTES (see A.4's per-segment-type audit for which
    types are verified so far).

    Source:
    N/A

    Inputs:
    segment              [RCAIDE.Framework.Mission.Segments.Segment] already-configured, not yet appended to a mission
    number_of_unknowns   [int]  mission + network unknowns per control point (caller-supplied -- see module docstring)
    max_dimension        [int]
    min_control_points   [int]
    tolerance            [float]
    step_size            [float]

    Outputs:
    list of segments, same class as the input, ready to append_segment in order in its place

    Properties Used:
    N/A
    """
    segment_class = type(segment)
    n_points      = segment.state.numerics.number_of_control_points

    points_per_subsegment, number_of_subsegments = compute_subsegment_layout(
        n_points, number_of_unknowns, max_dimension=max_dimension, min_control_points=min_control_points
    )

    if number_of_subsegments == 1:
        return [segment]

    if segment_class in LINEAR_PAIR_EXTENT_ATTRIBUTES:
        start_attr, end_attr = LINEAR_PAIR_EXTENT_ATTRIBUTES[segment_class]
        v0 = getattr(segment, start_attr)
        vf = getattr(segment, end_attr)
        edges = np.linspace(v0, vf, number_of_subsegments + 1)
        piece_bounds = [(edges[i], edges[i + 1]) for i in range(number_of_subsegments)]
    elif segment_class in DIVIDE_EXTENT_ATTRIBUTES:
        attr  = DIVIDE_EXTENT_ATTRIBUTES[segment_class]
        total = getattr(segment, attr)
        piece_bounds = [(total / number_of_subsegments, None)] * number_of_subsegments
    else:
        raise ValueError(
            f"hp_decompose_segment doesn't know {segment_class.__name__}'s extent attribute(s) -- "
            f"not yet registered in LINEAR_PAIR_EXTENT_ATTRIBUTES/DIVIDE_EXTENT_ATTRIBUTES. "
            f"See A.4's per-segment-type audit in RCAIDE_compute_acceleration_path.md."
        )

    import copy
    pieces = []
    for i, (lo, hi) in enumerate(piece_bounds):
        piece = copy.deepcopy(segment)
        piece.tag = f"{segment.tag}_{i}"
        piece.state.numerics.number_of_control_points = points_per_subsegment
        piece.state.numerics.mission_solver.tolerance  = tolerance
        piece.state.numerics.mission_solver.step_size  = step_size

        if segment_class in LINEAR_PAIR_EXTENT_ATTRIBUTES:
            setattr(piece, start_attr, lo)
            setattr(piece, end_attr, hi)
        else:
            setattr(piece, attr, lo)

        pieces.append(piece)

    return pieces
