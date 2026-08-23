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
# Segment classes that hp_decompose_segment knows how to split. Each class
# maps to a list of extent specs, all applied together (some segment types --
# e.g. Constant_Mach_Linear_Altitude -- vary two attributes in lockstep, like
# altitude and distance, to keep a derived quantity like climb angle
# consistent). Each spec is one of:
#   ('pair', start_attr, end_attr)  -- linearly interpolated across sub-segments
#   ('divide', attr)                -- a total-extent scalar, divided evenly
#
# Only segment types whose extent attributes were directly confirmed against
# their own initialize_conditions code (this session's audit, spot-checked
# again while registering) belong here. Excluded on purpose:
#   - the four "needs-care" categories from the A.4 audit (altitude-as-
#     unknown re-seeding, heading continuity, Takeoff/Landing's velocity-as-
#     unknown structure) -- registering them here would silently apply the
#     plain linear-interpolation path where it's known to be insufficient.
#   - Ground/Battery_Discharge, Battery_Recharge -- the audit assumed
#     separate Library-side files for these that turned out not to exist
#     (only Battery_Charge_Discharge.py does); not re-verified yet.
#   - Single_Point (all 4) and Untrimmed -- not decomposition candidates at
#     all (hardcoded to 1-2 control points, no extent to split).
# See RCAIDE_compute_acceleration_path.md's A.4 section for the full audit.
EXTENT_ATTRIBUTES = {}


def _register(segment_class, *specs):
    EXTENT_ATTRIBUTES[segment_class] = specs


def _register_known_segment_types():
    # Imported lazily to avoid a hard import-order dependency between this
    # module and RCAIDE.Framework.Mission.Segments at package-init time.
    from RCAIDE.Framework.Mission.Segments import Cruise, Climb, Descent, Vertical_Flight

    # Cruise
    _register(Cruise.Constant_Acceleration_Constant_Altitude,
              ('pair', 'air_speed_start', 'air_speed_end'))
    _register(Cruise.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude,
              ('pair', 'air_speed_start', 'air_speed_end'), ('pair', 'pitch_initial', 'pitch_final'))
    _register(Cruise.Constant_Dynamic_Pressure_Constant_Altitude, ('divide', 'distance'))
    _register(Cruise.Constant_Dynamic_Pressure_Constant_Altitude_Loiter, ('divide', 'time'))
    _register(Cruise.Constant_Mach_Constant_Altitude, ('divide', 'distance'))
    _register(Cruise.Constant_Mach_Constant_Altitude_Loiter, ('divide', 'time'))
    _register(Cruise.Constant_Pitch_Rate_Constant_Altitude, ('pair', 'pitch_initial', 'pitch_final'))
    _register(Cruise.Constant_Speed_Constant_Altitude, ('divide', 'distance'))
    _register(Cruise.Constant_Speed_Constant_Altitude_Loiter, ('divide', 'time'))
    # Curved_Constant_Radius_Constant_Speed_Constant_Altitude excluded: needs-care
    # (true_course continuity isn't carried by state.initials like everything else).

    # Climb
    _register(Climb.Constant_Acceleration_Constant_Pitchrate_Constant_Angle,
              ('pair', 'air_speed_start', 'air_speed_end'),
              ('pair', 'altitude_start', 'altitude_end'),
              ('pair', 'pitch_initial', 'pitch_final'))
    _register(Climb.Constant_CAS_Constant_Rate, ('pair', 'altitude_start', 'altitude_end'))
    _register(Climb.Constant_Dynamic_Pressure_Constant_Rate, ('pair', 'altitude_start', 'altitude_end'))
    _register(Climb.Constant_EAS_Constant_Rate, ('pair', 'altitude_start', 'altitude_end'))
    _register(Climb.Constant_Mach_Constant_Rate, ('pair', 'altitude_start', 'altitude_end'))
    _register(Climb.Constant_Mach_Linear_Altitude,
              ('pair', 'altitude_start', 'altitude_end'), ('divide', 'distance'))
    _register(Climb.Constant_Speed_Constant_Angle, ('pair', 'altitude_start', 'altitude_end'))
    _register(Climb.Constant_Speed_Constant_Rate, ('pair', 'altitude_start', 'altitude_end'))
    _register(Climb.Constant_Speed_Linear_Altitude,
              ('pair', 'altitude_start', 'altitude_end'), ('divide', 'distance'))
    _register(Climb.Constant_Throttle_Constant_Speed, ('pair', 'altitude_start', 'altitude_end'))
    _register(Climb.Linear_Mach_Constant_Rate,
              ('pair', 'altitude_start', 'altitude_end'), ('pair', 'mach_number_start', 'mach_number_end'))
    _register(Climb.Linear_Speed_Constant_Rate,
              ('pair', 'altitude_start', 'altitude_end'), ('pair', 'air_speed_start', 'air_speed_end'))
    # Constant_Dynamic_Pressure_Constant_Angle, Constant_Mach_Constant_Angle excluded:
    # needs-care (interior altitude is a solved unknown re-seeded from initialize_conditions).

    # Descent (mirrors Climb)
    _register(Descent.Constant_CAS_Constant_Rate, ('pair', 'altitude_start', 'altitude_end'))
    _register(Descent.Constant_EAS_Constant_Rate, ('pair', 'altitude_start', 'altitude_end'))
    _register(Descent.Constant_Speed_Constant_Angle, ('pair', 'altitude_start', 'altitude_end'))
    _register(Descent.Constant_Speed_Constant_Rate, ('pair', 'altitude_start', 'altitude_end'))
    _register(Descent.Constant_Throttle_Constant_Speed, ('pair', 'altitude_start', 'altitude_end'))
    _register(Descent.Linear_Mach_Constant_Rate,
              ('pair', 'altitude_start', 'altitude_end'), ('pair', 'mach_number_start', 'mach_number_end'))
    _register(Descent.Linear_Speed_Constant_Rate,
              ('pair', 'altitude_start', 'altitude_end'), ('pair', 'air_speed_start', 'air_speed_end'))

    # Vertical_Flight
    _register(Vertical_Flight.Climb, ('pair', 'altitude_start', 'altitude_end'))
    _register(Vertical_Flight.Descent, ('pair', 'altitude_start', 'altitude_end'))
    _register(Vertical_Flight.Hover, ('divide', 'time'))

    # Ground (Takeoff/Landing: needs-care, velocity/time are solved unknowns;
    # Battery_Discharge/Recharge: audit's assumed file location doesn't
    # exist, not re-verified) -- none registered this pass.


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
    segment's class is registered in EXTENT_ATTRIBUTES (see A.4's per-
    segment-type audit for which types are verified so far).

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

    specs = EXTENT_ATTRIBUTES.get(segment_class)
    if specs is None:
        raise ValueError(
            f"hp_decompose_segment doesn't know {segment_class.__name__}'s extent attribute(s) -- "
            f"not yet registered in EXTENT_ATTRIBUTES. See A.4's per-segment-type audit in "
            f"RCAIDE_compute_acceleration_path.md."
        )

    # Precompute each spec's per-piece values once, up front, from the
    # original (undivided) segment -- pieces are built from these, not from
    # each other, so there's no compounding rounding error across the chain.
    per_spec_values = []
    for spec in specs:
        if spec[0] == 'pair':
            _, start_attr, end_attr = spec
            v0 = getattr(segment, start_attr)
            vf = getattr(segment, end_attr)
            edges = np.linspace(v0, vf, number_of_subsegments + 1)
            per_spec_values.append(('pair', start_attr, end_attr, edges))
        elif spec[0] == 'divide':
            _, attr = spec
            total = getattr(segment, attr)
            per_spec_values.append(('divide', attr, total / number_of_subsegments))
        else:
            raise ValueError(f"Unknown extent spec kind {spec[0]!r} for {segment_class.__name__}")

    import copy
    pieces = []
    for i in range(number_of_subsegments):
        piece = copy.deepcopy(segment)
        piece.tag = f"{segment.tag}_{i}"
        piece.state.numerics.number_of_control_points = points_per_subsegment
        piece.state.numerics.mission_solver.tolerance  = tolerance
        piece.state.numerics.mission_solver.step_size  = step_size

        for value in per_spec_values:
            if value[0] == 'pair':
                _, start_attr, end_attr, edges = value
                setattr(piece, start_attr, edges[i])
                setattr(piece, end_attr, edges[i + 1])
            else:
                _, attr, per_piece_value = value
                setattr(piece, attr, per_piece_value)

        pieces.append(piece)

    return pieces
