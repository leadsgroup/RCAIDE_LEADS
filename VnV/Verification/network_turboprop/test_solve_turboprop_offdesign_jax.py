# VnV/Verification/network_turboprop/test_solve_turboprop_offdesign_jax.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.Turboprop_OffDesign_Matching import (
    pack_turboprop_design_constants, pack_turboprop_reference_point, solve_turboprop_offdesign_kernel,
    solve_turboprop_offdesign_robust)
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.solve_turboprop_offdesign_jax import (
    turboprop_matching_step, turboprop_matching_outputs, solve_turboprop_offdesign_jax)
import numpy as np
import sys
import os

sys.path.append(os.path.join(os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from ATR_72 import vehicle_setup

# ----------------------------------------------------------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    turboprop = list(list(vehicle_setup().networks)[0].propulsors)[0]

    # the matching equations written for NumPy and JAX reproduce the numba kernel exactly
    check_step_matches_numba_kernel(turboprop)

    try:
        import jax
    except ImportError:
        print("JAX is not installed (pip install RCAIDE[ad]); JAX checks skipped")
        return
    jax.config.update("jax_enable_x64", True)

    # values, jit and derivatives, including the propeller's actuator disk
    check_jax_solve(turboprop)

    # points no solver matches (combustor exit below the inlet stagnation temperature) are NaN, not a wrong number
    check_unmatched_points_are_nan(turboprop)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Points: Mach, static temperature [K], static pressure [Pa], Tt4 fraction, shaft power offtake [W]
# ----------------------------------------------------------------------------------------------------------------------
def takeoff_climb_and_cruise_points():
    return np.array([[0.05, 288.15, 101325.0, 1.00, 2e4],
                     [0.20, 288.15, 101325.0, 0.90, 2e4],
                     [0.40, 268.66, 70121.1,  0.70, 2e4],
                     [0.45, 242.70, 41105.2,  0.90, 2e4],
                     [0.45, 242.70, 41105.2,  0.50, 0.0]])

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_step_matches_numba_kernel(turboprop):
    design_constants = turboprop.offdesign_matching.design_constants
    reference_point  = turboprop.offdesign_matching.reference_point
    design           = pack_turboprop_design_constants(design_constants)
    reference        = pack_turboprop_reference_point(reference_point)
    for M0, T0, P0, fraction, offtake in takeoff_climb_and_cruise_points():
        Tt4    = fraction * reference_point.Tt4
        inputs = tuple(np.array([value]) for value in (M0, T0, P0, Tt4, offtake))

        # the kernel's own iteration and stopping test on tau_c and tau_tL, after the first step
        tau_tL = lambda pi_tL: 1 - design_constants.eta_tL * (1 - pi_tL ** ((design_constants.gamma_t - 1) / design_constants.gamma_t))
        state  = tuple(np.array([value]) for value in reference[[4, 6, 9]])
        previous_tau_tL = np.array([reference_point.tau_tL])
        for i in range(200):
            new_state, collapsed = turboprop_matching_step(state, inputs, design, reference)
            settled = i > 0 and abs(tau_tL(new_state[1][0]) - previous_tau_tL[0]) < 1e-8 and abs(new_state[0][0] - state[0][0]) < 1e-8
            previous_tau_tL = tau_tL(new_state[1])
            state  = new_state
            if settled:
                break
        # the kernel recomputes the gas generator once more after its loop: one more step
        state, collapsed = turboprop_matching_step(state, inputs, design, reference)
        outputs = turboprop_matching_outputs(state, inputs, design, reference)
        kernel  = solve_turboprop_offdesign_kernel(design, reference, M0, T0, P0, Tt4, 1e-8, 200, np.nan, np.nan, np.nan, offtake)

        # kernel output order: tau_c 4, pi_tL 7, mass flow 10, thrust 12, fuel flow 13, V9 19, shaft power 25
        for value, index in [(state[0], 4), (state[1], 7), (outputs.mass_flow_rate, 10), (outputs.thrust, 12),
                             (outputs.fuel_mass_flow_rate, 13), (outputs.core_nozzle_exit_velocity, 19), (outputs.shaft_power, 25)]:
            assert value[0] == kernel[index], \
                f"xp matching differs from the numba kernel at output {index}: {value[0]} vs {kernel[index]}"
    print('xp matching step and outputs equal the numba kernel exactly')


def check_jax_solve(turboprop):
    import jax.numpy as jnp
    from RCAIDE.Library.Methods.Utilities.derivative_check import check_derivatives
    design_constants = turboprop.offdesign_matching.design_constants
    reference_point  = turboprop.offdesign_matching.reference_point

    def thrust_fuel_and_power(M0, T0, P0, Tt4, offtake):
        if isinstance(M0, np.ndarray):
            # the NumPy reference: the robust solver point by point, tight enough that finite differences resolve it
            results = [solve_turboprop_offdesign_robust(design_constants, reference_point, M0[i], T0[i], P0[i], Tt4[i],
                                                        tolerance=1e-14, max_iterations=2000, shaft_power_offtake=offtake[i])
                       for i in range(len(M0))]
            return np.concatenate([[result[field] for result in results] for field in ('thrust', 'fuel_mass_flow_rate', 'shaft_power')])
        outputs, matched = solve_turboprop_offdesign_jax(design_constants, reference_point, M0, T0, P0, Tt4, offtake)
        return jnp.concatenate([outputs.thrust, outputs.fuel_mass_flow_rate, outputs.shaft_power])

    points = takeoff_climb_and_cruise_points()
    inputs = (points[:, 0], points[:, 1], points[:, 2], points[:, 3] * reference_point.Tt4, points[:, 4])
    report = check_derivatives(thrust_fuel_and_power, inputs, value_tolerance=1e-10, derivative_tolerance=1e-6)
    print(f'JAX turboprop matching: values {report.value_error:.1e}, compiled {report.compiled_error:.1e}, '
          f'JVP vs FD {report.jvp_error:.1e}, JVP vs VJP {report.vjp_consistency_error:.1e}')
    assert report.passed, report


def check_unmatched_points_are_nan(turboprop):
    import jax.numpy as jnp
    design_constants = turboprop.offdesign_matching.design_constants
    reference_point  = turboprop.offdesign_matching.reference_point
    points  = np.array([[0.45, 242.70, 41105.2, 0.9, 0.0],
                        [0.45, 242.70, 41105.2, 0.1, 0.0]])
    outputs, matched = solve_turboprop_offdesign_jax(design_constants, reference_point, *(jnp.asarray(points[:, k]) for k in range(3)),
                                                     jnp.asarray(points[:, 3] * reference_point.Tt4), jnp.asarray(points[:, 4]))
    assert bool(matched[0]) and not bool(matched[1])
    assert np.isfinite(outputs.thrust[0]) and np.isnan(outputs.thrust[1])
    print('unmatched point returns NaN')


if __name__ == '__main__':
    main()
