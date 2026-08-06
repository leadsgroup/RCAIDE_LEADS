import RCAIDE
import numpy as np
import warnings
from tqdm import tqdm

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
    following iteration to catch it on.
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
            last_state = segment.state

            segment.mission_tag = mission.tag

            original_expand = segment.process.initialize.expand_state
            segment.process.initialize.expand_state(segment)
            segment.process.initialize.expand_state = RCAIDE.Library.Methods.skip

            segment.evaluate()
            segment.process.initialize.expand_state = original_expand

            if segment.converged is False:
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
