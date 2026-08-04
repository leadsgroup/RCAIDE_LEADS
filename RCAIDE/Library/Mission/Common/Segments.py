import RCAIDE
import numpy as np
import warnings
from tqdm import tqdm

def sequential_segments(mission):
    print(r"""
          +----------------------------------------------------+
          |              MISSION SOLVER INITIATED              |
          +----------------------------------------------------+
          """)
    segments = list(mission.segments.items())
    last_state = None

    # bar_format includes {desc} on the left
    bar_format = (
        "{desc} |{bar}| "
        "{n_fmt}/{total_fmt} segs "
        "[{elapsed}<{remaining}, {rate_fmt}]"
    )

    # start out green
    with tqdm(total=len(segments),
              bar_format=bar_format,
              colour="green",
              unit="seg") as pbar:

        error_flag = False
        for tag, segment in segments:
            # update the {desc} field
            pbar.set_description(f"Solving {segment.tag}")

            # carry over state
            if last_state is not None:
                segment.state.initials = last_state
            last_state = segment.state

            # tag it
            segment.mission_tag = mission.tag

            # do the init/skip dance
            original_expand = segment.process.initialize.expand_state
            segment.process.initialize.expand_state(segment)
            segment.process.initialize.expand_state = RCAIDE.Library.Methods.skip

            segment.evaluate()
            segment.process.initialize.expand_state = original_expand

            # Check this segment's own convergence immediately, not the
            # previous segment's on the next iteration -- the old check based
            # on segment.state.initials never caught the last segment in the
            # mission at all, since there's no following iteration to catch it.
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
