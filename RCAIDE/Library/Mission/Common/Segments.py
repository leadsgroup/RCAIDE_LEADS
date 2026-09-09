import RCAIDE
import numpy as np
import warnings
from tqdm import tqdm

def seed_unknowns_from_previous_piece(segment, previous_state):
    """Seeds an hp-decomposed piece's initial unknown guess from the previous
    piece's converged final control point.

    Without this, every piece of a decomposed segment starts from the same
    static default guess (e.g. thrust_vector_angle=0.5 rad at every control
    point of every piece), regardless of where in the original segment's
    extent that piece actually falls. Found investigating a real SLSQP
    convergence failure (A.4): a piece mid-transition can be far from that
    flat default even though its neighbors, closer to the original segment's
    own start/end, aren't -- IPOPT tolerated it, scipy's SLSQP got stuck at
    its iteration limit without converging.

    Only touches unknown names present in both pieces (mission/network group
    unknowns are named identically across pieces of the same decomposed
    segment, since they all share the same assigned_control_variables/
    network topology) -- silently skips anything that doesn't match rather
    than raising, since a mismatch here means this segment isn't actually a
    same-type sibling piece.

    Parameters
    ----------
    segment        : the piece about to be solved (state.unknowns already
                      populated with the static default guess)
    previous_state  : the previous segment's state, already evaluated/
                       converged
    """
    for group in ('mission', 'network'):
        this_group = segment.state.unknowns[group]
        prev_group = previous_state.unknowns[group]
        for key in this_group.keys():
            if key == 'tag' or key not in prev_group:
                continue
            prev_final_value = np.atleast_2d(prev_group[key])[-1, 0]
            this_group[key][:] = prev_final_value


def sequential_segments(mission):
    """
    Evaluates each segment in a mission in order, chaining state between them.

    Parameters
    ----------
    mission : RCAIDE.Framework.Mission.Mission
        Mission whose segments (mission.segments) are evaluated in order

    Returns
    -------
    None

    Notes
    -----
    Each segment's final state is carried over as the next segment's
    ``state.initials``, so downstream segments start from where the previous
    one left off. ``expand_state`` is called once per segment to allocate that
    segment's state arrays, then swapped out for a no-op (``RCAIDE.Library.
    Methods.skip``) for the duration of ``segment.evaluate()`` so it isn't
    redundantly re-run by any nested call, and restored afterward.

    Progress is reported on a tqdm bar (green; turns red the first time any
    segment fails to converge, and stays red for the rest of the mission).
    A segment that fails to converge does not stop the mission -- it emits a
    warning and evaluation continues, since downstream segments may still be
    useful to inspect even though their results (and anything depending on
    the failed segment) may not be physically valid.

    Each segment's own convergence is checked immediately after it evaluates,
    rather than checking the previous segment's convergence via
    ``segment.state.initials`` on the following iteration -- that older
    approach never caught the last segment in the mission, since there is no
    following iteration to catch it on. The check reads
    ``segment.state.numerics.mission_solver.converged``, which always exists
    (defaulting to ``None``), rather than ``segment.converged``, which segment
    types with no iterative solver step never set.

    A segment with ``state.numerics.hp_decomposition.seed_guess_from_previous_
    piece`` set (only ever true for pieces 1..K-1 of an hp-decomposed segment,
    set by ``hp_decompose_segment``) has its initial unknown guess seeded from
    the previous piece's converged final control point, instead of keeping
    whatever static default guess ``set_mission_residuals_and_unknowns``/
    ``set_network_residuals_and_unknowns`` assigned it. Ordinary (non-
    decomposed) segments never have this flag set, so their behavior is
    unchanged. See ``seed_unknowns_from_previous_piece``'s docstring.

    That seeding is skipped when the previous piece did NOT converge --
    seeding an unknown from a piece that itself failed hands the next piece a
    starting guess with no particular relationship to a real trim point
    (whatever SLSQP's last iterate happened to be), which was observed to
    cascade the same "Singular matrix C in LSQ subproblem" failure through
    every subsequent piece of a decomposed segment even when the first
    piece's own failure was an isolated, otherwise-recoverable case. Skipping
    the seed falls back to the piece's own static default guess instead.
    ``state.initials`` (the physical boundary state: position, altitude,
    velocity, battery charge, etc.) is still always carried forward
    regardless of convergence -- unlike the unknown guess, it is a required
    continuity condition, not just a search starting point, so there is no
    safe fallback if it's wrong; the warning above is the only guard against
    that case.
    """
    print(r"""
          +----------------------------------------------------+
          |              MISSION SOLVER INITIATED              |
          +----------------------------------------------------+
          """)
    segments = list(mission.segments.items())
    last_state = None

    bar_format = (
        "{desc} |{bar}| "
        "{n_fmt}/{total_fmt} segs "
        "[{elapsed}<{remaining}, {rate_fmt}]"
    )

    with tqdm(total=len(segments),
              bar_format=bar_format,
              colour="green",
              unit="seg") as pbar:

        error_flag = False
        for tag, segment in segments:
            pbar.set_description(f"Solving {segment.tag}")

            if last_state is not None:
                segment.state.initials = last_state
                if (segment.state.numerics.hp_decomposition.seed_guess_from_previous_piece
                        and last_state.numerics.mission_solver.converged is not False):
                    seed_unknowns_from_previous_piece(segment, last_state)
            last_state = segment.state

            segment.mission_tag = mission.tag

            original_expand = segment.process.initialize.expand_state
            segment.process.initialize.expand_state(segment)
            segment.process.initialize.expand_state = RCAIDE.Library.Methods.skip

            segment.evaluate()
            segment.process.initialize.expand_state = original_expand

            if segment.state.numerics.mission_solver.converged is False:
                if not error_flag:
                    pbar.colour = "red"
                    error_flag = True
                warnings.warn(
                    f"Mission segment '{segment.tag}' failed to converge; "
                    f"its results (and any downstream segments/analyses that "
                    f"depend on them) may not be physically valid.",
                    stacklevel=2)
            segment.state.number_of_mission_residuals = 0
            segment.state.number_of_mission_unknowns  = 0
            segment.state.number_of_network_residuals = 0
            segment.state.number_of_network_unknowns  = 0
            pbar.update(1)
