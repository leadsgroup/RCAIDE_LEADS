# VnV/Verification/network_turbojet/test_solve_turbojet_offdesign_jax.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.Turbojet_OffDesign_Matching import (
    pack_turbojet_design_constants, pack_turbojet_reference_point, solve_turbojet_offdesign_kernel,
    solve_turbojet_offdesign_robust, apply_turbojet_afterburner)
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.solve_turbojet_offdesign_jax import (
    turbojet_matching_step, turbojet_matching_outputs, solve_turbojet_offdesign_jax)
import numpy as np
import sys
import os

sys.path.append(os.path.join(os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Concorde import vehicle_setup

# ----------------------------------------------------------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    turbojet = list(list(vehicle_setup().networks)[0].propulsors)[0]

    # the matching equations written for NumPy and JAX reproduce the numba kernel exactly
    check_step_matches_numba_kernel(turbojet)

    try:
        import jax
    except ImportError:
        print("JAX is not installed (pip install RCAIDE[ad]); JAX checks skipped")
        return
    jax.config.update("jax_enable_x64", True)

    # values, jit and derivatives, dry and with the afterburner lit
    check_jax_solve(turbojet, afterburner=False)
    check_jax_solve(turbojet, afterburner=True)

    # points no solver matches (combustor exit below the inlet stagnation temperature) are NaN, not a wrong number
    check_unmatched_points_are_nan(turbojet)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Points: Mach, static temperature [K], static pressure [Pa], Tt4 fraction, shaft power offtake [W]
# ----------------------------------------------------------------------------------------------------------------------
def climb_and_cruise_points():
    return np.array([[0.30, 288.15, 101325.0, 0.90, 0.0],
                     [0.90, 255.68, 54048.3,  0.70, 0.0],
                     [1.30, 216.65, 22699.9,  1.00, 0.0],
                     [2.00, 216.65, 10287.5,  0.90, 0.0],
                     [2.00, 216.65, 7565.2,   0.70, 0.0]])

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_step_matches_numba_kernel(turbojet):
    design_constants = turbojet.offdesign_matching.design_constants
    reference_point  = turbojet.offdesign_matching.reference_point
    design           = pack_turbojet_design_constants(design_constants)
    reference        = pack_turbojet_reference_point(reference_point)
    for M0, T0, P0, fraction, offtake in climb_and_cruise_points():
        Tt4    = fraction * reference_point.Tt4
        inputs = tuple(np.array([value]) for value in (M0, T0, P0, Tt4, offtake))

        # the kernel's own iteration and stopping test: tau_c and tau_tL change less than 1e-8, after the first step
        state = tuple(np.array([value]) for value in reference[[4, 5, 6, 10]])
        for i in range(200):
            new_state, collapsed = turbojet_matching_step(state, inputs, design, reference)
            settled = i > 0 and abs(new_state[1][0] - state[1][0]) < 1e-8 and abs(new_state[0][0] - state[0][0]) < 1e-8
            state   = new_state
            if settled:
                break
        outputs = turbojet_matching_outputs(state, inputs, design, reference)
        kernel  = solve_turbojet_offdesign_kernel(design, reference, M0, T0, P0, Tt4, 1e-8, 200, 0.5, np.nan, np.nan, np.nan, offtake)

        # kernel output order: tau_c 6, tau_tL 8, pi_tL 9, mass flow 12, thrust 14, fuel flow 15, V9 20, T9 21
        for value, index in [(state[0], 6), (state[1], 8), (state[2], 9), (outputs.mass_flow_rate, 12), (outputs.thrust, 14),
                             (outputs.fuel_mass_flow_rate, 15), (outputs.core_nozzle_exit_velocity, 20),
                             (outputs.core_nozzle_exit_static_temperature, 21)]:
            assert value[0] == kernel[index], f"xp matching differs from the numba kernel at output {index}: {value[0]} vs {kernel[index]}"
    print('xp matching step and outputs equal the numba kernel exactly')


def check_jax_solve(turbojet, afterburner):
    import jax.numpy as jnp
    from RCAIDE.Library.Methods.Utilities.derivative_check import check_derivatives
    design_constants = turbojet.offdesign_matching.design_constants
    reference_point  = turbojet.offdesign_matching.reference_point

    def thrust_and_fuel(M0, T0, P0, Tt4, offtake):
        if isinstance(M0, np.ndarray):
            # the NumPy reference: the robust solver point by point, tight enough that finite differences resolve it
            results = []
            for i in range(len(M0)):
                result = solve_turbojet_offdesign_robust(design_constants, reference_point, M0[i], T0[i], P0[i], Tt4[i],
                                                         tolerance=1e-14, max_iterations=2000, shaft_power_offtake=offtake[i])
                if afterburner:
                    result = apply_turbojet_afterburner(result, design_constants, turbojet.afterburner, turbojet.working_fluid, T0[i], P0[i])
                results.append(result)
            return np.concatenate([[float(result.thrust) for result in results], [float(result.fuel_mass_flow_rate) for result in results]])
        outputs, matched = solve_turbojet_offdesign_jax(design_constants, reference_point, M0, T0, P0, Tt4, offtake)
        if afterburner:
            outputs = apply_turbojet_afterburner(outputs, design_constants, turbojet.afterburner, turbojet.working_fluid, T0, P0)
        return jnp.concatenate([outputs.thrust, outputs.fuel_mass_flow_rate])

    points = climb_and_cruise_points()
    inputs = (points[:, 0], points[:, 1], points[:, 2], points[:, 3] * reference_point.Tt4, points[:, 4])
    report = check_derivatives(thrust_and_fuel, inputs, value_tolerance=1e-10, derivative_tolerance=1e-6)
    print(f'JAX turbojet matching, afterburner {afterburner}: values {report.value_error:.1e}, compiled {report.compiled_error:.1e}, '
          f'JVP vs FD {report.jvp_error:.1e}, JVP vs VJP {report.vjp_consistency_error:.1e}')
    assert report.passed, report


def check_unmatched_points_are_nan(turbojet):
    import jax.numpy as jnp
    design_constants = turbojet.offdesign_matching.design_constants
    reference_point  = turbojet.offdesign_matching.reference_point
    points  = np.array([[2.0, 216.65, 10287.5, 0.9, 0.0],
                        [2.0, 216.65, 10287.5, 0.2, 0.0]])
    outputs, matched = solve_turbojet_offdesign_jax(design_constants, reference_point, *(jnp.asarray(points[:, k]) for k in range(3)),
                                                    jnp.asarray(points[:, 3] * reference_point.Tt4), jnp.asarray(points[:, 4]))
    assert bool(matched[0]) and not bool(matched[1])
    assert np.isfinite(outputs.thrust[0]) and np.isnan(outputs.thrust[1])
    print('unmatched point returns NaN')


if __name__ == '__main__':
    main()
