# VnV/Verification/powertrain/test_turbofan_offdesign_matching.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
import pytest

from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_OffDesign_Matching import (
    solve_turbofan_offdesign_kernel, KERNEL_OUTPUT_FIELDS, mfp, area_from_mass_flow_rate, compressor_pressure_ratio,
    nozzle_state, mass_flow_parameter_kernel, area_from_mass_flow_rate_kernel, constant_efficiency_pressure_ratio_kernel,
    nozzle_state_kernel)

# ----------------------------------------------------------------------------------------------------------------------
#  Inputs: packed design constants and reference point of the GE90-94B in VnV/Validation/propulsors
# ----------------------------------------------------------------------------------------------------------------------
DESIGN    = np.array([1.4014209633903467, 1.3182824077016748, 1002.1447612195284, 1188.9339309408233, 0.98, 4.302e7,
                      0.997, 0.94, 0.2262809301802349, 0.7162998351843716, 0.995, 0.995, 0.8512162553995772,
                      0.9018946688380691, 0.9093219564309437, 0.6368876946260631, 0.9836362949683074, 0.0])
REFERENCE = np.array([0.8, 218.92391647450165, 23908.734083913052, 1430.0, 1.2417312433357439, 0.7056814935887487,
                      0.22931572865881644, 8.5, 20.003, 2.596257276100993, 1.9908, 1.0, 1.0, 517.91060837295])

NAN = np.nan
#        (M0,  T0,     P0,       Tt4,    tolerance, max_iterations, relaxation, guess tau_f, tau_tL, pi_tL, offtake)
CASES = {'design point':        (0.8, REFERENCE[1], REFERENCE[2], 1430., 1e-8, 200, 0.5, NAN, NAN,  NAN,  NAN),
         'sea-level takeoff':   (0.0, 288.15, 101325., 1717., 1e-8, 200, 0.5, NAN, NAN,  NAN,  NAN),
         'supersonic inlet':    (1.2, 216.65, 19330.,  1430., 1e-8, 200, 0.5, NAN, NAN,  NAN,  NAN),
         'guess and offtake':   (0.8, REFERENCE[1], REFERENCE[2], 1430., 1e-8, 200, 0.5, 1.2, 0.7, 0.23, 2e5),
         'not converged':       (0.8, REFERENCE[1], REFERENCE[2], 1430., 1e-8, 1,   0.5, NAN, NAN,  NAN,  NAN),
         'collapsed nozzle':    (0.0, 288.15, 101325., 500.,  1e-8, 200, 0.5, NAN, NAN,  NAN,  NAN)}


def python_version(kernel):
    """Plain-Python body of a numba kernel (traceable by coverage); the function itself without numba."""
    return getattr(kernel, 'py_func', kernel)


def solve(condition, kernel=solve_turbofan_offdesign_kernel):
    return dict(zip(KERNEL_OUTPUT_FIELDS, kernel(DESIGN, REFERENCE, *CASES[condition])))


# ----------------------------------------------------------------------------------------------------------------------
#  Helper kernels against their plain-Python counterparts
# ----------------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("mach_number", [0.3, 0.8, 1.0])
def test_mass_flow_parameter_kernel(mach_number):
    assert python_version(mass_flow_parameter_kernel)(mach_number, 1.4, 287.0) == pytest.approx(mfp(mach_number, 1.4, 287.0), rel=1e-14)


@pytest.mark.parametrize("mach_number", [0.3, 0.8, 1.0])
def test_area_from_mass_flow_rate_kernel(mach_number):
    expected = area_from_mass_flow_rate(100.0, 50000.0, 600.0, 287.0, 1.33, mach_number)
    assert python_version(area_from_mass_flow_rate_kernel)(100.0, 50000.0, 600.0, 287.0, 1.33, mach_number) == pytest.approx(expected, rel=1e-14)


@pytest.mark.parametrize("pressure_ratio", [0.9, 1.5, 3.0])   # collapsed, unchoked, choked
def test_nozzle_state_kernel(pressure_ratio):
    assert python_version(nozzle_state_kernel)(pressure_ratio, 1.4) == pytest.approx(nozzle_state(pressure_ratio, 1.4), rel=1e-14)


@pytest.mark.parametrize("temperature_ratio", [1.3, -5.0])     # normal, floored base
def test_constant_efficiency_pressure_ratio_kernel(temperature_ratio):
    expected, _ = compressor_pressure_ratio(temperature_ratio, 0.9, 1.4)
    assert python_version(constant_efficiency_pressure_ratio_kernel)(temperature_ratio, 0.9, 1.4) == pytest.approx(expected, rel=1e-14)


# ----------------------------------------------------------------------------------------------------------------------
#  Off-design kernel
# ----------------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("condition", CASES)
def test_python_matches_compiled(condition):
    compiled = solve(condition)
    python   = solve(condition, python_version(solve_turbofan_offdesign_kernel))
    for field in KERNEL_OUTPUT_FIELDS:
        np.testing.assert_allclose(python[field], compiled[field], rtol=1e-10, equal_nan=True, err_msg=field)


def test_design_point_recovers_reference_state():
    result = solve('design point')
    assert result['converged'] == 1.0 and result['collapsed'] == 0.0
    assert result['tau_f']          == pytest.approx(REFERENCE[4], rel=1e-8)
    assert result['tau_tL']         == pytest.approx(REFERENCE[5], rel=1e-8)
    assert result['mass_flow_rate'] == pytest.approx(REFERENCE[13], rel=1e-8)


@pytest.mark.parametrize("condition", ['sea-level takeoff', 'supersonic inlet', 'guess and offtake'])
def test_off_design_converges_with_positive_thrust(condition):
    result = solve(condition)
    assert result['converged'] == 1.0 and result['collapsed'] == 0.0
    assert result['thrust'] > 0 and result['specific_fuel_consumption'] > 0


def test_takeoff_thrust_exceeds_cruise_thrust():
    assert solve('sea-level takeoff')['thrust'] > solve('design point')['thrust']


def test_iteration_limit_reports_not_converged():
    result = solve('not converged')
    assert result['converged'] == 0.0 and result['iterations'] == 1.0


def test_collapsed_nozzle_reports_no_thrust():
    result = solve('collapsed nozzle')
    assert result['collapsed'] == 1.0 and result['converged'] == 0.0
    assert np.isnan(result['thrust'])
    assert result['convergence_delta'] == np.inf
