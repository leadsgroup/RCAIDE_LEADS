# VnV/Verification/derivatives/test_mission_solver_jacobian_jax.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # The root finder of a segment can take the segment's compiled Jacobian in place of finite differences
    # (numerics.mission_solver.jacobian = "compiled"). Here a tiltrotor vertical climb and departure transition are
    # solved both ways, and the two solves must converge to the same unknowns. The solver mirrors unknowns back into
    # their bounds, so the slope of that mirroring, which scales the compiled Jacobian, is checked on its own.
    try:
        import jax
    except ImportError:
        print("JAX is not installed (pip install RCAIDE[ad]); JAX checks skipped")
        return
    jax.config.update("jax_enable_x64", True)

    vnv_folder   = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    start_folder = os.getcwd()
    sys.path.insert(0, os.path.join(vnv_folder, "Vehicles"))
    os.chdir(os.path.join(vnv_folder, "Verification", "network_vtol"))
    sys.path.insert(0, os.getcwd())

    finite_difference = solve_mission("finite_difference")
    compiled          = solve_mission("compiled")

    check_reflection_slope()
    check_compiled_jacobian_used(compiled)
    check_same_solution(finite_difference, compiled, tolerance=1e-6)

    os.chdir(start_folder)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_reflection_slope():
    from RCAIDE.Library.Mission.Solver.solver import reflect_into_bounds, reflection_slope

    # the solver mirrors unknowns back into their bounds, and the Jacobian carries the slope of that mirroring:
    # two-sided, lower-only, upper-only and unbounded unknowns, inside and outside their bounds
    lower = np.array([0., 0., 0., 0., 0.,    1., 1.,    -np.inf, -np.inf, -np.inf])
    upper = np.array([1., 1., 1., 1., 1., np.inf, np.inf,     2.,      2.,  np.inf])
    x     = np.array([0.3, -0.2, 1.4, 2.6, -1.3, 1.5, 0.2,   1.5,     2.7,     5.])
    step  = 1e-6
    slope = (reflect_into_bounds(x + step, lower, upper) - reflect_into_bounds(x - step, lower, upper)) / (2 * step)
    error = np.max(np.abs(reflection_slope(x, lower, upper) - slope))
    print(f"  slope of the mirroring into bounds against central differences: {error:.1e}")
    assert error <= 1e-8, f"reflection_slope differs from central differences by {error}"
    return


def check_compiled_jacobian_used(mission):
    from RCAIDE.Library.Mission.Solver.compiled_residual import COMPILED_SEGMENTS

    # every root-finder segment compiled its residual, and the registry holds them
    segments = root_finder_segments(mission)
    assert len(segments) > 0, "the mission has no root-finder segment"
    assert len(COMPILED_SEGMENTS) > 0, "no segment residual was compiled"
    print(f"  {len(segments)} root-finder segments solved with {len(COMPILED_SEGMENTS)} compiled residuals")
    return


def check_same_solution(finite_difference, compiled, tolerance):
    for a, b in zip(root_finder_segments(finite_difference), root_finder_segments(compiled)):
        assert a.converged and b.converged, f"{a.tag}: converged {a.converged} with finite differences, {b.converged} with the compiled Jacobian"
        xa    = a.state.unknowns.pack_array()
        xb    = b.state.unknowns.pack_array()
        error = np.max(np.abs(xa - xb) / np.maximum(1e-12, np.abs(xa)))
        print(f"  {a.tag}: {len(xa)} unknowns, largest relative difference {error:.1e}")
        assert error <= tolerance, f"{a.tag}: unknowns differ by {error}"
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Mission
# ----------------------------------------------------------------------------------------------------------------------
def solve_mission(jacobian):
    import vtol_aircraft_test as vtol

    vehicle  = vtol.TR_vehicle_setup(False)
    analyses = vtol.TR_analyses_setup(vtol.TR_configs_setup(vehicle))
    mission  = vtol.missions_setup(vtol.TR_mission_setup(analyses)).base_mission
    for segment in root_finder_segments(mission):
        segment.state.numerics.mission_solver.jacobian = jacobian
    mission.evaluate()
    return mission


def root_finder_segments(mission):
    return [segment for segment in mission.segments.values() if segment.state.numerics.mission_solver.type == "root_finder"]


if __name__ == '__main__':
    main()
