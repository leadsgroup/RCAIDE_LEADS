# RCAIDE/Library/Mission/Common/Pre_Process/merge_hp_decomposed_segments.py
#
#
# Created:  Aug 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import copy
import numpy as np

from RCAIDE.Framework.Analyses                      import Process
from RCAIDE.Framework.Mission.Common.Conditions      import Conditions
from RCAIDE.Framework.Mission.Common.State           import append_array
from RCAIDE.Library.Mission.Solver.hp_decomposition  import EXTENT_ATTRIBUTES

# ----------------------------------------------------------------------------------------------------------------------
#  merge_hp_decomposed_segments
# ----------------------------------------------------------------------------------------------------------------------
def and_ignoring_none(values):
    """AND across a run of hp-decomposed pieces' *_solver.converged flags --
    None (never solved) propagates rather than counting as either True or
    False, matching the fact that an ordinary, non-decomposed segment's own
    converged flag also defaults to None (Numerics.py).
    """
    values = list(values)
    if any(v is None for v in values):
        return None
    return all(values)


def extent_attribute_names(specs):
    """Attribute names hp_decompose_segment actually overwrites per piece,
    for each extent spec kind registered in EXTENT_ATTRIBUTES (RCAIDE.
    Library.Mission.Solver.hp_decomposition) -- not the per-kind partition
    math itself, just which names to restore from the pristine original.
    'cumulative' only touches its target_attr; the source_attr it reads
    from is a plain total, never modified on any piece.
    """
    names = []
    for spec in specs:
        kind = spec[0]
        if kind == 'pair':
            names.extend([spec[1], spec[2]])
        elif kind in ('divide', 'cumulative'):
            names.append(spec[1])
    return names


def merge_hp_decomposed_segments(mission):
    """
    Reassembles hp-decomposed segments back into one reported segment per
    original tag, once every piece has been solved.

    hp-decomposition (RCAIDE.Library.Mission.Solver.hp_decomposition, wired
    in via Pre_Process.hp_decompose_mission) splits a segment like "cruise"
    into "cruise_0", "cruise_1", ... purely as a solving-time trick -- it
    doesn't change what the mission "is" from a results-consumer's point of
    view. Without this step, mission.segments holds the pieces under their
    split tags instead of the original one, so results.segments.cruise (the
    normal way every verification test, and any real user script, reads
    mission results) breaks.

    Detects a run of pieces via state.numerics.hp_decomposition.original_tag/
    piece_index/piece_count, set only by hp_decompose_mission on pieces it
    actually created -- never inferred from tag text, since a real segment
    can legitimately be named e.g. "cruise_2" on its own (see VnV/
    Verification/analysis_stability/trimmed_flight_test.py) and a text match
    would silently merge unrelated segments together.

    The reported segment is built from a deep copy of piece 0, not the
    pristine pre-split segment stashed at state.numerics.hp_decomposition.
    original_segment: piece 0 already went through the mission's full
    process.initialize (geometry/aero/stability/emissions/mass_properties),
    which runs after hp_decompose_mission splits the segment, so it's the
    one with a real, populated .analyses (e.g. the VLM vortex_distribution
    settings VnV/Verification/analysis_aerodynamics/VLM_aerodynamics_test.py
    reads) -- the pristine original never went through any of that. Only the
    extent attributes that actually differ per piece (altitude, distance,
    ...) get restored from the pristine original, by name, via
    extent_attribute_names -- this still doesn't need to duplicate
    hp_decompose_segment's per-kind partition math in reverse, just know
    which attribute names it touched.

    state.conditions/unknowns/residuals are replaced with the pieces' values
    concatenated along the control-point axis, in piece order, via
    Conditions.do_recursive/append_array (only rank-2, per-control-point
    array leaves are stacked; everything else is left as piece 0's value).
    numerics.*_solver.converged becomes the AND across pieces;
    numerics.number_of_control_points becomes the pieces' total; the
    now-mismatched-size Chebyshev dimensionless/time operator matrices
    (sized to piece 0's own smaller point count) are cleared rather than
    left stale, since nothing re-solves the merged, reported segment.

    Must run after sequential_segments (process.converge): pieces don't have
    solved state.conditions until then.

    Parameters
    ----------
    mission : RCAIDE.Framework.Mission.Mission
        Mission whose segments (mission.segments) were just solved

    Returns
    -------
    None -- mutates mission.segments in place when any run is merged

    Properties Used:
    N/A
    """
    original_segments = list(mission.segments.items())
    new_container      = Process()
    any_merged         = False

    i = 0
    n = len(original_segments)
    while i < n:
        tag, segment = original_segments[i]
        hp = segment.state.numerics.hp_decomposition

        if hp.original_tag is None:
            new_container.append(segment)
            i += 1
            continue

        piece_count      = hp.piece_count
        pieces           = [seg for _, seg in original_segments[i:i + piece_count]]
        original_segment = hp.original_segment

        merged_segment     = copy.deepcopy(pieces[0])
        merged_segment.tag = hp.original_tag

        specs = EXTENT_ATTRIBUTES.get(type(merged_segment), ())
        for name in extent_attribute_names(specs):
            setattr(merged_segment, name, getattr(original_segment, name))

        for key in ('conditions', 'unknowns', 'residuals'):
            merged = Conditions()
            for j, piece in enumerate(pieces):
                if j == 0:
                    merged.update(piece.state[key])
                else:
                    merged = merged.do_recursive(append_array, piece.state[key])
            merged_segment.state[key] = merged

        # Segment.__defaults__ aliases segment.conditions to segment.state.
        # conditions at construction time (RCAIDE.Framework.Mission.Segments.
        # Segment); copy.deepcopy above preserved that aliasing, but replacing
        # state.conditions wholesale just above broke it -- restore it so
        # results.segments.<tag>.conditions (the form every verification test
        # and any real user script reads) sees the merged data too, not piece
        # 0's stale, pre-replacement copy.
        merged_segment.conditions = merged_segment.state.conditions

        merged_numerics = merged_segment.state.numerics
        merged_numerics.number_of_control_points = sum(
            piece.state.numerics.number_of_control_points for piece in pieces)
        merged_numerics.mission_solver.converged = and_ignoring_none(
            piece.state.numerics.mission_solver.converged for piece in pieces)
        merged_numerics.network_solver.converged = and_ignoring_none(
            piece.state.numerics.network_solver.converged for piece in pieces)
        merged_numerics.dimensionless.control_points = np.empty([0,0])
        merged_numerics.dimensionless.differentiate  = np.empty([0,0])
        merged_numerics.dimensionless.integrate      = np.empty([0,0])
        merged_numerics.time.control_points          = np.empty([0,0])
        merged_numerics.time.differentiate           = np.empty([0,0])
        merged_numerics.time.integrate               = np.empty([0,0])
        merged_numerics.hp_decomposition.original_tag                       = None
        merged_numerics.hp_decomposition.piece_index                        = None
        merged_numerics.hp_decomposition.piece_count                        = None
        merged_numerics.hp_decomposition.original_segment                   = None
        merged_numerics.hp_decomposition.seed_guess_from_previous_piece     = False

        new_container.append(merged_segment)
        any_merged = True
        i += piece_count

    if any_merged:
        mission.segments = new_container

    return
