# RCAIDE/Library/Mission/Common/Pre_Process/hp_decompose_mission.py
#
#
# Created:  Aug 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import copy

from RCAIDE.Framework.Analyses                     import Process
from RCAIDE.Library.Mission.Solver.hp_decomposition import hp_decompose_segment, EXTENT_ATTRIBUTES
from .geometry                                      import geometry
from .energy                                        import energy
from .set_mission_residuals_and_unknowns            import set_mission_residuals_and_unknowns
from .set_network_residuals_and_unknowns            import set_network_residuals_and_unknowns

# ----------------------------------------------------------------------------------------------------------------------
#  hp_decompose_mission
# ----------------------------------------------------------------------------------------------------------------------
class SegmentListView:
    """Minimal mission-like stand-in so energy()/set_mission_residuals_and_
    unknowns()/set_network_residuals_and_unknowns() -- which only ever read
    mission.segments -- can run against a throwaway single-segment list
    instead of a real Mission object."""
    def __init__(self, segments):
        self.segments = segments


def hp_decompose_mission(mission):
    """Applies A.4's static hp-decomposition (RCAIDE.Library.Mission.Solver.
    hp_decomposition) to every eligible segment in the mission before the
    rest of process.initialize runs.

    A segment is eligible when its class is registered in EXTENT_ATTRIBUTES
    and segment.state.numerics.hp_decomposition.enabled is True (Numerics.py
    default: True; set False on a segment to opt it out). Eligible segments
    are replaced in mission.segments by their decomposed pieces, in order;
    everything else passes through unchanged.

    Must run first in process.initialize, before geometry/energy/mass_
    properties/aero/stability/emissions/set_*_residuals_and_unknowns: those
    steps allocate arrays sized by state.numerics.number_of_control_points,
    so they need to see each piece's own (smaller) point count, not the
    original segment's. Decomposing first and letting the existing sequence
    run once, on the already-expanded list, means none of those steps need
    to change or be re-run per piece.

    hp_decompose_segment needs number_of_unknowns up front, before this
    segment has been through the very steps that would normally count it
    (energy()'s topology analysis feeds set_network_residuals_and_unknowns).
    Resolved by running those steps once on a throwaway deepcopy purely to
    read off number_of_mission_unknowns + number_of_network_unknowns, then
    discarding it -- the unknown *count* only depends on the segment's
    assigned_control_variables and the vehicle's network topology, neither
    of which depends on number_of_control_points, so the count learned from
    the N=original prototype is exactly the count each smaller piece has too.

    Assumptions:
    See hp_decompose_segment's own docstring for the tolerance/step_size and
    max_dimension/min_control_points rationale (Numerics.py's hp_decomposition
    block).

    Source:
    N/A

    Inputs:
    mission

    Outputs:
    None -- mutates mission.segments in place when any segment is split

    Properties Used:
    N/A
    """
    original_segments = list(mission.segments.items())
    new_container      = Process()
    any_split          = False

    prototypes = {tag: copy.deepcopy(segment) for tag, segment in original_segments
                  if segment.state.numerics.hp_decomposition.enabled and type(segment) in EXTENT_ATTRIBUTES}

    if prototypes:
        geometry(SegmentListView(list(prototypes.values())))

    for tag, segment in original_segments:
        numerics = segment.state.numerics
        if tag not in prototypes:
            new_container.append(segment)
            continue

        prototype = prototypes[tag]
        energy(SegmentListView([prototype]))
        set_mission_residuals_and_unknowns(SegmentListView([prototype]))
        set_network_residuals_and_unknowns(SegmentListView([prototype]))
        number_of_unknowns = prototype.state.number_of_mission_unknowns + prototype.state.number_of_network_unknowns

        pieces = hp_decompose_segment(
            segment, number_of_unknowns,
            tolerance = numerics.hp_decomposition.tolerance,
            step_size = numerics.hp_decomposition.step_size,
        )
        if len(pieces) > 1:
            any_split = True
            for i, piece in enumerate(pieces):
                piece.state.numerics.hp_decomposition.enabled      = False
                piece.state.numerics.hp_decomposition.original_tag = segment.tag
                piece.state.numerics.hp_decomposition.piece_index  = i
                piece.state.numerics.hp_decomposition.piece_count  = len(pieces)
                
            pieces[0].state.numerics.hp_decomposition.original_segment = segment
        for piece in pieces:
            new_container.append(piece)

    if any_split:
        mission.segments = new_container

    return
