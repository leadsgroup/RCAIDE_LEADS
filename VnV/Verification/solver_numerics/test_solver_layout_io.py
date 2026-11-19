# VnV/Verification/solver_numerics/test_solver_layout_io.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

from RCAIDE.Framework.Mission.Common      import State, Segment_Layout
from RCAIDE.Library.Mission.Solver.solver import Segment_Problem, solver_slices

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # The mission solver reads and writes a segment through slices of its Segment_Layout buffer.
    # A slice covers the unknowns or residuals of the mission and, when present, of the network.
    N = 4   # number of control points
    check_solver_slices_with_and_without_network(N)
    check_segment_problem_reads_and_writes_the_layout(N)
    check_non_adjacent_groups_rejected(N)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_solver_slices_with_and_without_network(N):
    state  = make_state(N)
    packed = {group: state.deep_get(group).pack_array() for group in ("unknowns.mission", "unknowns.network",
                                                                      "residuals.mission", "residuals.network")}
    layout = Segment_Layout(state)

    # with the network, mission entries come first and network entries follow
    unknowns, residuals = solver_slices(layout, network=True)
    assert np.array_equal(layout.buffer[unknowns], np.concatenate([packed["unknowns.mission"], packed["unknowns.network"]]))
    assert np.array_equal(layout.buffer[residuals], np.concatenate([packed["residuals.mission"], packed["residuals.network"]]))

    # without it, only the mission entries
    unknowns, residuals = solver_slices(layout, network=False)
    assert np.array_equal(layout.buffer[unknowns], packed["unknowns.mission"])
    assert np.array_equal(layout.buffer[residuals], packed["residuals.mission"])
    return


def check_segment_problem_reads_and_writes_the_layout(N):
    # Segment_Problem is the Nexus problem the optimizer sees for one segment
    state   = make_state(N)
    layout  = Segment_Layout(state)
    problem = Segment_Problem()
    problem.layout                       = layout
    problem.unknown_slice, problem.residual_slice = solver_slices(layout, network=True)
    problem.residual_index               = {f"Residual_{i + 1}": i for i in range(2 * N)}
    problem.postprocess.nothing          = 0

    # the optimizer's inputs reach the unknowns in the state
    problem.set_inputs(None, np.arange(2.0 * N))
    assert np.array_equal(state.unknowns.network.rpm[:, 0], np.arange(N, 2.0 * N))

    # the optimizer's constraints come from the residuals in the state
    state.residuals.network.torque[...] = 7.0
    values = problem.output_values([["Residual_1", "=", 0], [f"Residual_{2 * N}", "=", 0], ["nothing", 1, 1]])
    assert np.array_equal(values, [-1.0, 7.0, 0.0])
    return


def check_non_adjacent_groups_rejected(N):
    # popping and re-inserting the mission group puts the network first, so one slice can no longer cover both in order
    state = make_state(N)
    state.unknowns.mission = state.unknowns.pop("mission")
    layout = Segment_Layout(state)
    try:
        solver_slices(layout, network=True)
    except ValueError:
        return
    raise AssertionError("solver_slices accepted groups in the wrong order")

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def make_state(N):
    """A small segment state with mission and network unknowns and residuals."""
    state = State()
    state.unknowns.mission.pitch_angle  = np.linspace(0.0, 0.1, N)[:, None]
    state.unknowns.network.rpm          = np.linspace(1.0, 2.0, N)[:, None]
    state.residuals.mission.force_x     = np.linspace(-1.0, 1.0, N)[:, None]
    state.residuals.network.torque      = np.arange(N, dtype=float)[:, None]
    return state


if __name__ == '__main__':
    main()
