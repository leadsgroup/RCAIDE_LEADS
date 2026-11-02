# VnV/Verification/solver_numerics/test_solver_layout_io.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
import pytest

from RCAIDE.Framework.Mission.Common      import State, Segment_Layout
from RCAIDE.Library.Mission.Solver.solver import Segment_Problem, solver_slices

N = 4


def make_state():
    state = State()
    state.unknowns.mission.pitch_angle  = np.linspace(0.0, 0.1, N)[:, None]
    state.unknowns.network.rpm          = np.linspace(1.0, 2.0, N)[:, None]
    state.residuals.mission.force_x     = np.linspace(-1.0, 1.0, N)[:, None]
    state.residuals.network.torque      = np.arange(N, dtype=float)[:, None]
    return state


def test_solver_slices_with_and_without_network():
    state  = make_state()
    packed = {g: state.deep_get(g).pack_array() for g in ("unknowns.mission", "unknowns.network",
                                                          "residuals.mission", "residuals.network")}
    layout = Segment_Layout(state)
    unknowns, residuals = solver_slices(layout, network=True)
    assert np.array_equal(layout.buffer[unknowns], np.concatenate([packed["unknowns.mission"], packed["unknowns.network"]]))
    assert np.array_equal(layout.buffer[residuals], np.concatenate([packed["residuals.mission"], packed["residuals.network"]]))
    unknowns, residuals = solver_slices(layout, network=False)
    assert np.array_equal(layout.buffer[unknowns], packed["unknowns.mission"])
    assert np.array_equal(layout.buffer[residuals], packed["residuals.mission"])


def test_segment_problem_reads_and_writes_the_layout():
    state   = make_state()
    layout  = Segment_Layout(state)
    problem = Segment_Problem()
    problem.layout                       = layout
    problem.unknown_slice, problem.residual_slice = solver_slices(layout, network=True)
    problem.residual_index               = {f"Residual_{i + 1}": i for i in range(2 * N)}
    problem.postprocess.nothing          = 0

    problem.set_inputs(None, np.arange(2.0 * N))
    assert np.array_equal(state.unknowns.network.rpm[:, 0], np.arange(N, 2.0 * N))
    state.residuals.network.torque[...] = 7.0
    values = problem.output_values([["Residual_1", "=", 0], [f"Residual_{2 * N}", "=", 0], ["nothing", 1, 1]])
    assert np.array_equal(values, [-1.0, 7.0, 0.0])


def test_non_adjacent_groups_rejected():
    state = make_state()
    state.unknowns.mission = state.unknowns.pop("mission")    # re-inserted, so network now comes first
    layout = Segment_Layout(state)
    with pytest.raises(ValueError):
        solver_slices(layout, network=True)
