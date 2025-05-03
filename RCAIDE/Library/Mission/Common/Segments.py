import RCAIDE
import numpy as np
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

            # the moment we see a non-converged segment, flip to red
            if segment.state.initials != {}:
                if not segment.state.initials.numerics.solver.converged and not error_flag:
                    pbar.colour = "red"
                    error_flag = True

            # do the init/skip dance
            segment.process.initialize.expand_state(segment)
            segment.process.initialize.expand_state = RCAIDE.Library.Methods.skip

            segment.evaluate()
            pbar.update(1)
