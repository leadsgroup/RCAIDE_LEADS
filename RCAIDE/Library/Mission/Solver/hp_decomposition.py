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
def compute_subsegment_layout(number_of_control_points, number_of_unknowns, max_dimension=64, min_control_points=4):
    """Static (design-time) hp-decomposition layout: how many sub-segments a
    segment should be split into, and how many control points each gets.

    Finds the SMALLEST divisor of number_of_control_points that (a) is at
    or above min_control_points and (b) keeps total sub-problem dimension
    (points-per-piece * number_of_unknowns) under max_dimension. Requiring
    an exact divisor keeps every sub-segment the same size -- no ragged
    last piece with fewer points than the rest.

    Smallest, not largest: calibration on a real vehicle (A.4 in
    RCAIDE_compute_acceleration_path.md) found control-point count itself,
    not the points*unknowns product, is what drives solve difficulty --
    tripling unknowns at fixed points (5->12, dim 20->48) cost ~30% wall-
    clock; doubling points at fixed unknowns (4->8, dim 20->40, so a
    *smaller* dimension) cost ~3x. So max_dimension is now mostly a safety
    net against a genuinely pathological control-variable count, not the
    primary lever -- min_control_points is. This trades fewer, larger sub-
    segments for more, smaller ones whenever both are available, which is
    the opposite of what an "avoid chaining overhead" instinct would pick,
    but matches the evidence.

    This is a pure function of static problem size (never of solver
    behavior/convergence outcome), so it returns the same layout every time
    for the same segment definition -- required for later JIT/AD reuse
    (Track B), where changing array shapes at runtime based on how a solve
    went would defeat trace reuse entirely.

    Assumptions:
    min_control_points=4 is calibration-backed (see A.4's floor discussion:
    cubic as the traditional minimum for real curvature, matches a pre-
    existing hand-tuned precedent in this codebase, converged correctly in
    testing). max_dimension=64 is NOT independently calibrated -- the sweep
    never found a U-driven wall at fixed small n (dimension up to 48 at n=4
    converged cleanly, barely slower than dimension 20), so 64 is set
    generous enough to stay non-binding across the validated range and only
    guard a genuinely pathological control-variable count.

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

    if number_of_control_points < min_control_points:
        # Already below the floor a split piece would need to meet -- there's
        # no way to divide this segment into >=1 pieces each with at least
        # min_control_points points without a piece smaller than the whole
        # segment itself, so leave it undecomposed rather than erroring.
        return number_of_control_points, 1

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

    points_per_subsegment = min(candidates)
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
#   ('pair', start_attr, end_attr)         -- linearly interpolated across sub-segments
#   ('divide', attr)                       -- a total-extent scalar, divided evenly
#   ('cumulative', target_attr, source_attr) -- piece i gets
#       segment.target_attr + i*(segment.source_attr / K); for segment types
#       (so far just the curved-radius cruise) where target_attr is read as a
#       plain, non-chained attribute rather than carried forward via
#       state.initials the way position/velocity/time are, so each piece
#       needs the cumulative effect of every prior piece added back in.
#
# Only segment types whose extent attributes were directly confirmed against
# their own initialize_conditions code (this session's audit, spot-checked
# again while registering) belong here. Excluded on purpose:
#   - the remaining "needs-care" categories from the A.4 audit (altitude-as-
#     unknown re-seeding, Takeoff/Landing's velocity-as-unknown structure,
#     Battery_Recharge's solve-dependent charging duration) -- registering
#     them here would silently apply the plain linear-interpolation path
#     where it's known to be insufficient.
#   - Single_Point (all 4) and Untrimmed -- not decomposition candidates at
#     all (hardcoded to 1-2 control points, no extent to split).
# See RCAIDE_compute_acceleration_path.md's A.4 section for the full audit.
EXTENT_ATTRIBUTES = {}


def register(segment_class, *specs):
    EXTENT_ATTRIBUTES[segment_class] = specs


def register_known_segment_types():
    # Imported lazily to avoid a hard import-order dependency between this
    # module and RCAIDE.Framework.Mission.Segments at package-init time.
    from RCAIDE.Framework.Mission.Segments import Cruise, Climb, Descent, Vertical_Flight, Ground

    # Cruise
    register(Cruise.Constant_Acceleration_Constant_Altitude,
              ('pair', 'air_speed_start', 'air_speed_end'))
    register(Cruise.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude,
              ('pair', 'air_speed_start', 'air_speed_end'), ('pair', 'pitch_initial', 'pitch_final'))
    register(Cruise.Constant_Dynamic_Pressure_Constant_Altitude, ('divide', 'distance'))
    register(Cruise.Constant_Dynamic_Pressure_Constant_Altitude_Loiter, ('divide', 'time'))
    register(Cruise.Constant_Mach_Constant_Altitude, ('divide', 'distance'))
    register(Cruise.Constant_Mach_Constant_Altitude_Loiter, ('divide', 'time'))
    register(Cruise.Constant_Pitch_Rate_Constant_Altitude, ('pair', 'pitch_initial', 'pitch_final'))
    register(Cruise.Constant_Speed_Constant_Altitude, ('divide', 'distance'))
    register(Cruise.Constant_Speed_Constant_Altitude_Loiter, ('divide', 'time'))
    # Curved_Constant_Radius_Constant_Speed_Constant_Altitude excluded: needs-care
    # (true_course continuity isn't carried by state.initials like everything else).

    # Climb
    register(Climb.Constant_Acceleration_Constant_Pitchrate_Constant_Angle,
              ('pair', 'air_speed_start', 'air_speed_end'),
              ('pair', 'altitude_start', 'altitude_end'),
              ('pair', 'pitch_initial', 'pitch_final'))
    register(Climb.Constant_CAS_Constant_Rate, ('pair', 'altitude_start', 'altitude_end'))
    register(Climb.Constant_Dynamic_Pressure_Constant_Rate, ('pair', 'altitude_start', 'altitude_end'))
    register(Climb.Constant_EAS_Constant_Rate, ('pair', 'altitude_start', 'altitude_end'))
    register(Climb.Constant_Mach_Constant_Rate, ('pair', 'altitude_start', 'altitude_end'))
    register(Climb.Constant_Mach_Linear_Altitude,
              ('pair', 'altitude_start', 'altitude_end'), ('divide', 'distance'))
    register(Climb.Constant_Speed_Constant_Angle, ('pair', 'altitude_start', 'altitude_end'))
    register(Climb.Constant_Speed_Constant_Rate, ('pair', 'altitude_start', 'altitude_end'))
    register(Climb.Constant_Speed_Linear_Altitude,
              ('pair', 'altitude_start', 'altitude_end'), ('divide', 'distance'))
    register(Climb.Constant_Throttle_Constant_Speed, ('pair', 'altitude_start', 'altitude_end'))
    register(Climb.Linear_Mach_Constant_Rate,
              ('pair', 'altitude_start', 'altitude_end'), ('pair', 'mach_number_start', 'mach_number_end'))
    register(Climb.Linear_Speed_Constant_Rate,
              ('pair', 'altitude_start', 'altitude_end'), ('pair', 'air_speed_start', 'air_speed_end'))
    # Constant_Dynamic_Pressure_Constant_Angle, Constant_Mach_Constant_Angle excluded:
    # needs-care (interior altitude is a solved unknown re-seeded from initialize_conditions).

    # Descent (mirrors Climb)
    register(Descent.Constant_CAS_Constant_Rate, ('pair', 'altitude_start', 'altitude_end'))
    register(Descent.Constant_EAS_Constant_Rate, ('pair', 'altitude_start', 'altitude_end'))
    register(Descent.Constant_Speed_Constant_Angle, ('pair', 'altitude_start', 'altitude_end'))
    register(Descent.Constant_Speed_Constant_Rate, ('pair', 'altitude_start', 'altitude_end'))
    register(Descent.Constant_Throttle_Constant_Speed, ('pair', 'altitude_start', 'altitude_end'))
    register(Descent.Linear_Mach_Constant_Rate,
              ('pair', 'altitude_start', 'altitude_end'), ('pair', 'mach_number_start', 'mach_number_end'))
    register(Descent.Linear_Speed_Constant_Rate,
              ('pair', 'altitude_start', 'altitude_end'), ('pair', 'air_speed_start', 'air_speed_end'))

    # Vertical_Flight
    register(Vertical_Flight.Climb, ('pair', 'altitude_start', 'altitude_end'))
    register(Vertical_Flight.Descent, ('pair', 'altitude_start', 'altitude_end'))
    register(Vertical_Flight.Hover, ('divide', 'time'))

    # Curved_Constant_Radius: true_course_control_points = segment.true_course
    # + t_nondim*turn_angle (initialize_conditions.py) -- true_course is read
    # directly, not carried forward via state.initials like everything else,
    # so each piece needs the heading already turned by prior pieces added
    # back in, not just an even turn_angle split.
    register(Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude,
              ('divide', 'turn_angle'), ('cumulative', 'true_course', 'turn_angle'))

    # Battery_Discharge: initialize_conditions.py's else-branch (non-Recharge)
    # reads segment.time directly -- a plain, prescribed duration, same as
    # Hover/the Loiter segments.
    register(Ground.Battery_Discharge, ('divide', 'time'))

    # Ground: the rest deliberately excluded, not just unfinished.
    #   - Takeoff/Landing: velocity profile and total elapsed time are both
    #     solved unknowns (not prescribed), a fundamentally different
    #     residual/unknown structure than every airborne segment here, and
    #     already have a reputation (independent of this feature) for being
    #     finicky to converge -- not worth compounding that with a first
    #     pass at decomposition. Revisit only with real motivation.
    #   - Battery_Recharge: charging duration is computed from cutoff_SOC and
    #     the SOC the segment actually starts at (initialize_conditions.py:
    #     "linear SOC increase" from state.initials's converged end-of-flight
    #     SOC, or initial_battery_conditions if it's the first segment) --
    #     not known until solve time, and specifically not known to
    #     hp_decompose_segment, which runs on a bare segment before it's
    #     chained into a mission (state.initials doesn't exist yet). Same
    #     class of problem as Takeoff/Landing, just for a different reason.


register_known_segment_types()


# ----------------------------------------------------------------------------------------------------------------------
#  hp_decompose_segment
# ----------------------------------------------------------------------------------------------------------------------
def hp_decompose_segment(segment, number_of_unknowns, tolerance, step_size,
                          max_dimension=None, min_control_points=None):
    """Splits one segment into a chain of smaller sub-segments of the same
    type, covering the same overall extent, per compute_subsegment_layout's
    static sizing rule.

    max_dimension/min_control_points default to the segment's own
    state.numerics.hp_decomposition.max_dimension/.min_control_points
    (Numerics.py) when not passed explicitly -- a sibling of mission_solver/
    network_solver, not nested under either, since total sub-problem
    dimension is mission + network unknowns together whenever
    network_solver.type is None, not a mission_solver-only concern. Pass
    either argument explicitly to override per-call.

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
    max_dimension        [int]  optional, defaults to segment.state.numerics.hp_decomposition.max_dimension
    min_control_points   [int]  optional, defaults to segment.state.numerics.hp_decomposition.min_control_points
    tolerance            [float]
    step_size            [float]

    Outputs:
    list of segments, same class as the input, ready to append_segment in order in its place

    Properties Used:
    N/A
    """
    if max_dimension is None:
        max_dimension = segment.state.numerics.hp_decomposition.max_dimension
    if min_control_points is None:
        min_control_points = segment.state.numerics.hp_decomposition.min_control_points

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
            if v0 is None or vf is None:
                return [segment]
            edges = np.linspace(v0, vf, number_of_subsegments + 1)
            per_spec_values.append(('pair', start_attr, end_attr, edges))
        elif spec[0] == 'divide':
            _, attr = spec
            total = getattr(segment, attr)
            per_spec_values.append(('divide', attr, total / number_of_subsegments))
        elif spec[0] == 'cumulative':
            _, target_attr, source_attr = spec
            base_value = getattr(segment, target_attr)
            per_piece  = getattr(segment, source_attr) / number_of_subsegments
            per_spec_values.append(('cumulative', target_attr, base_value, per_piece))
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
        # only pieces after the first: the first piece has no predecessor of
        # its own to seed from (state.initials already chains it to whatever
        # segment came before the original, undecomposed segment)
        piece.state.numerics.hp_decomposition.seed_guess_from_previous_piece = (i > 0)

        for value in per_spec_values:
            if value[0] == 'pair':
                _, start_attr, end_attr, edges = value
                setattr(piece, start_attr, edges[i])
                setattr(piece, end_attr, edges[i + 1])
            elif value[0] == 'divide':
                _, attr, per_piece_value = value
                setattr(piece, attr, per_piece_value)
            elif value[0] == 'cumulative':
                _, target_attr, base_value, per_piece = value
                setattr(piece, target_attr, base_value + i * per_piece)

        pieces.append(piece)

    return pieces
