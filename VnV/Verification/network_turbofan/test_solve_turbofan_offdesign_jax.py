# VnV/Verification/network_turbofan/test_solve_turbofan_offdesign_jax.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_OffDesign_Matching import (
    pack_design_constants, pack_reference_point, solve_turbofan_offdesign_kernel, solve_turbofan_offdesign_robust)
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.solve_turbofan_offdesign_jax import (
    turbofan_matching_step, turbofan_matching_outputs, solve_turbofan_offdesign_jax)
import numpy as np
import sys
import os

sys.path.append(os.path.join(os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Embraer_190 import vehicle_setup

# ----------------------------------------------------------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    turbofan = list(vehicle_setup().networks.fuel.propulsors)[0]

    # the matching equations written for NumPy and JAX reproduce the numba kernel exactly
    check_step_matches_numba_kernel(turbofan)

    try:
        import jax
    except ImportError:
        print("JAX is not installed (pip install RCAIDE[ad]); JAX checks skipped")
        return
    jax.config.update("jax_enable_x64", True)

    # values, jit and derivatives at points the JAX iteration solves directly
    check_jax_solve(turbofan, cruise_and_climb_points())

    # points the JAX iteration cannot solve from the reference point: the robust solver's continuation finds them
    check_jax_solve(turbofan, continuation_points())

    # points no solver matches (combustor exit below the inlet stagnation temperature) are NaN, not a wrong number
    check_unmatched_points_are_nan(turbofan)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Points: Mach, static temperature [K], static pressure [Pa], Tt4 fraction, shaft power offtake [W]
# ----------------------------------------------------------------------------------------------------------------------
def cruise_and_climb_points():
    return np.array([[0.78, 218.9, 23908.7, 1.00,  0.0],
                     [0.78, 218.9, 23908.7, 0.80,  0.0],
                     [0.50, 249.2, 54048.3, 0.90,  2e5],
                     [0.30, 268.7, 79501.4, 1.05,  2e5],
                     [0.20, 288.15, 101325., 1.10, 0.0]])

def continuation_points():
    # a motor adds shaft power at low throttle near sea level
    return np.array([[0.0, 288.15, 101325., 0.5, -1e6],
                     [0.1, 320.0,  101325., 0.5, -3e6],
                     [0.2, 288.15, 101325., 0.5, -1e6]])

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_step_matches_numba_kernel(turbofan):
    design_constants = turbofan.offdesign_matching.design_constants
    reference_point  = turbofan.offdesign_matching.reference_point
    design           = pack_design_constants(design_constants)
    reference        = pack_reference_point(reference_point)
    for M0, T0, P0, fraction, offtake in cruise_and_climb_points():
        Tt4    = fraction * reference_point.Tt4
        inputs = tuple(np.array([value]) for value in (M0, T0, P0, Tt4, offtake))

        # the kernel's own iteration and stopping test: tau_f and tau_tL change less than 1e-8, after the first step
        state = tuple(np.array([value]) for value in reference[[4, 5, 6, 7, 13]])
        for i in range(200):
            new_state, collapsed = turbofan_matching_step(state, inputs, design, reference)
            settled = i > 0 and abs(new_state[1][0] - state[1][0]) < 1e-8 and abs(new_state[0][0] - state[0][0]) < 1e-8
            state   = new_state
            if settled:
                break
        outputs = turbofan_matching_outputs(state, inputs, design, reference)
        kernel  = solve_turbofan_offdesign_kernel(design, reference, M0, T0, P0, Tt4, 1e-8, 200, 0.5, np.nan, np.nan, np.nan, offtake)

        # kernel output order: tau_f 6, tau_tL 10, pi_tL 11, alpha 12, mass flow 17, thrust 19, fuel flow 20, V9 25, V19 26
        for value, index in [(state[0], 6), (state[1], 10), (state[2], 11), (state[3], 12), (outputs['mass_flow_rate'], 17),
                             (outputs['thrust'], 19), (outputs['fuel_mass_flow_rate'], 20),
                             (outputs['core_nozzle_exit_velocity'], 25), (outputs['fan_nozzle_exit_velocity'], 26)]:
            # equal to the last bits: numba may fuse multiply-adds on some platforms (seen on Linux x86)
            assert abs(value[0] - kernel[index]) <= 1e-13 * abs(kernel[index]), \
                f"xp matching differs from the numba kernel at output {index}: {value[0]} vs {kernel[index]}"
    print('xp matching step and outputs equal the numba kernel to the last bits')


def check_jax_solve(turbofan, points):
    import jax.numpy as jnp
    from RCAIDE.Library.Methods.Utilities.derivative_check import check_derivatives
    design_constants = turbofan.offdesign_matching.design_constants
    reference_point  = turbofan.offdesign_matching.reference_point

    def thrust_and_fuel(M0, T0, P0, Tt4, offtake):
        if isinstance(M0, np.ndarray):
            # the NumPy reference: the robust solver point by point, tight enough that finite differences resolve it
            results = [solve_turbofan_offdesign_robust(design_constants, reference_point, M0[i], T0[i], P0[i], Tt4[i],
                                                       tolerance=1e-14, max_iterations=2000,
                                                       shaft_power_offtake=offtake[i]) for i in range(len(M0))]
            return np.concatenate([[result.thrust for result in results], [result.fuel_mass_flow_rate for result in results]])
        outputs, matched = solve_turbofan_offdesign_jax(design_constants, reference_point, M0, T0, P0, Tt4, offtake)
        return jnp.concatenate([outputs['thrust'], outputs['fuel_mass_flow_rate']])

    inputs = (points[:, 0], points[:, 1], points[:, 2], points[:, 3] * reference_point.Tt4, points[:, 4])
    report = check_derivatives(thrust_and_fuel, inputs, value_tolerance=1e-10, derivative_tolerance=1e-6)
    print(f'JAX matching on {len(points)} points: values {report.value_error:.1e}, compiled {report.compiled_error:.1e}, '
          f'JVP vs FD {report.jvp_error:.1e}, JVP vs VJP {report.vjp_consistency_error:.1e}')
    assert report.passed, report


def check_unmatched_points_are_nan(turbofan):
    import jax.numpy as jnp
    design_constants = turbofan.offdesign_matching.design_constants
    reference_point  = turbofan.offdesign_matching.reference_point
    points  = np.array([[0.78, 218.9, 23908.7, 1.0, 0.0],
                        [0.78, 218.9, 23908.7, 0.1, 0.0]])
    outputs, matched = solve_turbofan_offdesign_jax(design_constants, reference_point, *(jnp.asarray(points[:, k]) for k in range(3)),
                                                    jnp.asarray(points[:, 3] * reference_point.Tt4), jnp.asarray(points[:, 4]))
    assert bool(matched[0]) and not bool(matched[1])
    assert np.isfinite(outputs['thrust'][0]) and np.isnan(outputs['thrust'][1])
    print('unmatched point returns NaN')


if __name__ == '__main__':
    main()
