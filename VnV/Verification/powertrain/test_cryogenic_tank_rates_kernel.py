# VnV/Verification/powertrain/test_cryogenic_tank_rates_kernel.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import copy
import numpy as np
import pytest
from scipy.optimize import brentq

import RCAIDE
from RCAIDE.Framework.Core.Physical_Constants import UNIVERSAL_GAS_CONSTANT
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_tank_heat_leak import (
    CYLINDER, CUBOID, compute_cryogenic_tank_heat_leak)
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.cryogenic_tank_rates_kernel import (
    DIAGNOSTICS, cryogenic_tank_rates, cryogenic_tank_rates_tables, cryogenic_tank_node_diagnostics,
    liquid_height, liquid_height_residual, environment_heat_leak, interface_heat, soft_temperature_barrier)
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_tank_performance import (
    tank_state_rates, compute_liquid_height, Q_liq_to_int)

from cryogenic_tank_performance_test import build_case

MAX_HEATER_POWER = 5.0e3


def python_version(kernel):
    """Plain-Python body of a numba kernel (traceable by coverage); the function itself without numba."""
    return getattr(kernel, 'py_func', kernel)


# ----------------------------------------------------------------------------------------------------------------------
#  LH2 tank at 35,000 ft, as cryogenic_tank_performance_test
# ----------------------------------------------------------------------------------------------------------------------
def build_tank_case(geometry='cylindrical', symmetric=False):
    lh2         = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    tank, state = build_case(lh2, 20.0, 2.0e5, 8, 2.0, 6.0, 2.0)
    if geometry == 'conformal':
        # box with the cylinder's gross volume, so both RHS see the same tank
        s                    = tank.inner_structure
        s.lengths.internal   = 6.0
        s.widths.internal    = 1.6
        s.heights.internal   = tank.volume_properties.gross_volume / (6.0 * 1.6)
        s.thickness          = 0.004
        s.lengths.external   = s.lengths.internal + 2 * (s.thickness + tank.insulation.thickness)
        s.widths.external    = s.widths.internal  + 2 * (s.thickness + tank.insulation.thickness)
        s.heights.external   = s.heights.internal + 2 * (s.thickness + tank.insulation.thickness)
        tank.geometry_type   = 'conformal'
    tank.xz_plane_symmetric = symmetric

    fuel            = tank.fuel
    tank_conditions = state.conditions.energy.sources[tank.tag]
    (T_l_lo, T_l_hi), (P_lo, P_hi) = fuel.property_table_range(phase='liquid')
    (T_g_lo, T_g_hi), _            = fuel.property_table_range(phase='vapor')
    bounds = (T_g_lo, T_g_hi, T_l_lo, T_l_hi, P_lo, P_hi)
    R      = UNIVERSAL_GAS_CONSTANT / fuel.molecular_weight
    tables = cryogenic_tank_rates_tables(tank, R, *bounds, MAX_HEATER_POWER)

    fs     = state.conditions.freestream
    t      = np.ravel(state.numerics.time.control_points)
    ones   = np.ones_like(t)
    inputs = np.ascontiguousarray(np.vstack([2.0e5 * ones, fs.temperature[:,0], fs.kinematic_viscosity[:,0],
                                             fs.prandtl_number[:,0], fs.thermal_conductivity[:,0], 0.0 * ones]))
    y0     = np.array([tank_conditions.ullage_mass[0,0], tank_conditions.fuel_mass[0,0],
                       tank_conditions.ullage_temperature[0,0], tank_conditions.fuel_temperature[0,0]])
    return tank, R, bounds, tables, t, inputs, y0


def regime_state(tank, bounds, y0, regime):
    """States that switch on each branch of the tank model."""
    y      = y0.copy()
    m_full = tank.design_full_liquid_mass
    if regime == 'heater':            # ullage below minimum_operating_pressure
        y[0] *= 0.2
    elif regime == 'vent':            # ullage above design_pressure
        y[0] *= 3.0
    elif regime == 'gate':            # liquid inside the availability smoothing band
        y[1] = 1.2e-3 * m_full
    elif regime == 'floor':           # liquid below the thermal floor
        y[1] = 5.0e-4 * m_full
    elif regime == 'out_of_range':    # temperatures past both table edges
        y[2] = bounds[1] + 5.0
        y[3] = bounds[2] - 1.0
    return y


def python_rates(tank, R, bounds, t_i, y, t, inputs):
    nodes       = [np.array([v]) for v in y]
    drivers     = [np.array([np.interp(t_i, t, row)]) for row in inputs]
    rates, diag = tank_state_rates(tank, tank.fuel, R, *nodes, *drivers, *bounds, MAX_HEATER_POWER)
    return np.array([r[0] for r in rates]), diag


REGIMES = ['nominal', 'heater', 'vent', 'gate', 'floor', 'out_of_range']


# ----------------------------------------------------------------------------------------------------------------------
#  Right-hand side: kernel against the Python tank_state_rates
# ----------------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("geometry, symmetric", [('cylindrical', False), ('cylindrical', True), ('conformal', False)])
@pytest.mark.parametrize("regime", REGIMES)
@pytest.mark.parametrize("refuel", [0.0, 1.0])
def test_rates_match_python_model(geometry, symmetric, regime, refuel):
    tank, R, bounds, tables, t, inputs, y0 = build_tank_case(geometry, symmetric)
    inputs[5] = refuel
    y   = regime_state(tank, bounds, y0, regime)
    t_i = 0.5 * (t[2] + t[3])

    rates, diagnostics, status = python_version(cryogenic_tank_rates)(t_i, y, t, inputs, *tables)
    expected, diag             = python_rates(tank, R, bounds, t_i, y, t, inputs)

    assert status == 0
    np.testing.assert_allclose(rates, expected, rtol=1e-9, atol=1e-14)
    for i, name in enumerate(DIAGNOSTICS):
        np.testing.assert_allclose(diagnostics[i], np.asarray(diag[name], dtype=float)[0], rtol=1e-9, atol=1e-14, err_msg=name)


@pytest.mark.parametrize("regime", REGIMES)
def test_rates_compiled_matches_python(regime):
    tank, R, bounds, tables, t, inputs, y0 = build_tank_case()
    y = regime_state(tank, bounds, y0, regime)
    compiled = cryogenic_tank_rates(t[3], y, t, inputs, *tables)
    python   = python_version(cryogenic_tank_rates)(t[3], y, t, inputs, *tables)
    np.testing.assert_allclose(compiled[0], python[0], rtol=1e-12)
    np.testing.assert_allclose(compiled[1], python[1], rtol=1e-12)
    assert compiled[2] == python[2] == 0


def test_regimes_switch_on():
    tank, R, bounds, tables, t, inputs, y0 = build_tank_case()
    rates = python_version(cryogenic_tank_rates)
    d = {regime: rates(t[3], regime_state(tank, bounds, y0, regime), t, inputs, *tables)[1] for regime in REGIMES}
    i = {name: k for k, name in enumerate(DIAGNOSTICS)}
    assert d['vent'][i['m_dot_vent']]         > 1e-3
    assert d['nominal'][i['m_dot_vent']]      < 1e-9
    assert d['gate'][i['heater_saturated']]   == 1.0
    assert 0 < d['gate'][i['m_dot_avail']]    < 1
    assert d['gate'][i['m_dot_deficit']]      > 0
    assert d['floor'][i['m_l_thermal_floored']] == 1.0
    assert d['out_of_range'][i['T_out_of_range']] == 1.0
    assert d['nominal'][i['T_out_of_range']]  == 0.0


def test_unsolvable_heat_leak_returns_status():
    tank, R, bounds, tables, t, inputs, y0 = build_tank_case()
    inputs[1] = np.nan                     # non-finite ambient temperature: no bracket for the heat-leak solve
    rates, diagnostics, status = python_version(cryogenic_tank_rates)(t[3], y0, t, inputs, *tables)
    assert status != 0
    assert np.all(rates == 0) and np.all(diagnostics == 0)
    assert cryogenic_tank_node_diagnostics(t, *(np.full_like(t, v) for v in y0), inputs, tables) is None


def test_unsolvable_ullage_heat_leak_returns_status(monkeypatch):
    # the liquid-side heat leak solves, the ullage-side one does not
    import RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.cryogenic_tank_rates_kernel as kernel
    calls = {'n': 0}
    def leak(*args):
        calls['n'] += 1
        return (100.0, 0) if calls['n'] == 1 else (0.0, 3)
    monkeypatch.setattr(kernel, 'environment_heat_leak', leak)
    tank, R, bounds, tables, t, inputs, y0 = build_tank_case()
    rates, diagnostics, status = python_version(cryogenic_tank_rates)(t[3], y0, t, inputs, *tables)
    assert status == 3 and np.all(rates == 0)


def test_node_diagnostics_match_python_model():
    tank, R, bounds, tables, t, inputs, y0 = build_tank_case()
    states   = [np.full_like(t, v) for v in y0]
    diag     = cryogenic_tank_node_diagnostics(t, *states, inputs, tables)
    _, check = tank_state_rates(tank, tank.fuel, R, *states, *inputs, *bounds, MAX_HEATER_POWER)
    for name in DIAGNOSTICS:
        np.testing.assert_allclose(np.asarray(diag[name], dtype=float), np.asarray(check[name], dtype=float),
                                   rtol=1e-9, atol=1e-14, err_msg=name)
    for name in ("T_out_of_range", "m_l_thermal_floored", "heater_saturated"):
        assert diag[name].dtype == bool


def test_tables_vent_time_constant_defaults_to_regulation():
    tank, R, bounds, tables, t, inputs, y0 = build_tank_case()
    tank.vent_time_constant = None
    params = cryogenic_tank_rates_tables(tank, R, *bounds, MAX_HEATER_POWER)[6]
    assert params[20] == tank.pressure_regulation_time_constant
    assert params[0]  == CYLINDER


# ----------------------------------------------------------------------------------------------------------------------
#  Scalar kernels
# ----------------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("fraction", [-0.1, 0.0, 1e-6, 0.3, 0.5, 0.97, 1.0, 1.2])
def test_liquid_height_matches_brentq(fraction):
    r, l   = 1.0, 4.0
    v_full = np.pi * r**2 * l + (4 / 3) * np.pi * r**3
    v      = fraction * v_full
    h      = python_version(liquid_height)(r, l, v)
    assert h == pytest.approx(compute_liquid_height(r, l, v)[0], abs=1e-10)
    assert liquid_height(r, l, v) == pytest.approx(h, abs=1e-12)
    v_clip = min(max(v, 1e-9), v_full - 1e-9)
    assert abs(python_version(liquid_height_residual)(h, r, l, v_clip)) < 1e-6


def test_environment_heat_leak_matches_cylinder_solve():
    args     = (0.05, 288.15, 20.0, 167.0, 0.02, 0.025, 1.5e-5, 2.1e-5, 0.71, 0.9)
    geometry = np.array([0.99, 5.0, 0.99, 0., 1.0, 0.99, 5.0, 0.])
    Q, status = python_version(environment_heat_leak)(CYLINDER, *args, geometry)
    assert status == 0
    assert Q == pytest.approx(compute_cryogenic_tank_heat_leak(*args, 1.0, 0.99, 5.0)[1], rel=1e-8)
    assert environment_heat_leak(CYLINDER, *args, geometry)[0] == pytest.approx(Q, rel=1e-12)


def test_environment_heat_leak_cuboid_and_edge_cases():
    args     = (0.05, 288.15, 20.0, 167.0, 0.02, 0.025, 1.5e-5, 2.1e-5, 0.71, 0.9)
    geometry = np.array([4.0, 2.0, 1.5, 0., 4.1, 2.1, 1.6, 0.005])
    leak     = python_version(environment_heat_leak)
    Q, status = leak(CUBOID, *args, geometry)
    assert status == 0 and Q > 0
    # no temperature difference: no heat flow
    equal = list(args); equal[2] = equal[1]
    Q0, status = leak(CUBOID, *equal, geometry)
    assert status == 0 and Q0 == pytest.approx(0.0, abs=1e-6)
    # non-finite ambient: no bracket
    bad = list(args); bad[1] = np.inf
    assert leak(CUBOID, *bad, geometry) == (0.0, 1)


def test_environment_heat_leak_reports_solver_failure(monkeypatch):
    import RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.cryogenic_tank_rates_kernel as kernel
    monkeypatch.setattr(kernel, 'brent_heat_leak', lambda *a: (0.0, 2))
    args     = (0.05, 288.15, 20.0, 167.0, 0.02, 0.025, 1.5e-5, 2.1e-5, 0.71, 0.9)
    geometry = np.array([0.99, 5.0, 0.99, 0., 1.0, 0.99, 5.0, 0.])
    assert python_version(environment_heat_leak)(CYLINDER, *args, geometry) == (0.0, 2)


def test_interface_heat_matches_python():
    args = (21.0, 20.3, 3.0, 1.5, 70.0, 9.7e3, 1.3e-5, 0.1)
    assert python_version(interface_heat)(*args) == pytest.approx(Q_liq_to_int(*args), rel=1e-12)
    assert interface_heat(*args) == pytest.approx(Q_liq_to_int(*args), rel=1e-12)


def test_soft_temperature_barrier():
    barrier = python_version(soft_temperature_barrier)
    assert abs(barrier(20.0, 15.0, 30.0, 0.1)) < 1e-3     # inside the range: negligible
    assert barrier(35.0, 15.0, 30.0, 0.1) < -4.9         # above: pulls down by the excess
    assert barrier(10.0, 15.0, 30.0, 0.1) > 4.9          # below: pushes up by the deficit
