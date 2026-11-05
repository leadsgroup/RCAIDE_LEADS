# VnV/Verification/powertrain/test_compute_cryogenic_tank_performance.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import importlib
import numpy as np
import pytest

import RCAIDE
from RCAIDE.Framework.Core import Data, Units
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.append_cryogenic_tank_conditions import append_cryogenic_tank_conditions

from cryogenic_tank_performance_test import build_case, build_state, build_tank

# the package __init__ shadows the module name with the function, so fetch the module itself for monkeypatching
performance = importlib.import_module('RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_tank_performance')
kernel      = importlib.import_module('RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.cryogenic_tank_rates_kernel')

OUTPUTS = ('ullage_mass', 'fuel_mass', 'ullage_temperature', 'fuel_temperature', 'pressure', 'vent_rate',
           'boil_off_flow_rate', 'heater_power', 'fuel_volume', 'ullage_volume', 'mass_flow_rate')


def lh2_case(chemical_power=2.0e5, n_nodes=8):
    return build_case(RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen(), 20.0, chemical_power, n_nodes, 2.0, 6.0, 2.0)


def solve(tank, state):
    performance.compute_cryogenic_tank_performance(tank, state, network=None)
    return state.conditions.energy.sources[tank.tag]


def liquid_volume(tank, m_l, T_l):
    return m_l / tank.fuel.cryogen_properties(np.atleast_1d(T_l), "Density (kg/m3)", phase='liquid')[0]


# ----------------------------------------------------------------------------------------------------------------------
#  Single control point
# ----------------------------------------------------------------------------------------------------------------------
def test_single_control_point_matches_python_model():
    tank  = build_tank(RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen(), 20.0, 2.0, 6.0)
    state = build_state(2, 3600., 2.0e5, 35000. * Units.ft)
    # one point in time: no integration, only the diagnostics at the initial state
    state._size                        = 1
    state.numerics.time.control_points = np.zeros((1, 1))
    for field in ('temperature', 'kinematic_viscosity', 'prandtl_number', 'thermal_conductivity'):
        state.conditions.freestream[field] = state.conditions.freestream[field][:1]
    state.conditions.energy.distributors['fuel_line'].outputs.power.chemical = 2.0e5 * np.ones((1, 1))
    segment       = Data()
    segment.state = state
    append_cryogenic_tank_conditions(tank, segment)

    tc = solve(tank, state)
    assert tc.fuel_mass.shape == (1, 1)
    m_dot_engine = 2.0e5 / tank.fuel.lower_heating_value
    assert tc.outputs.power.chemical[0, 0] == pytest.approx(2.0e5, rel=1e-9)
    assert tc.mass_flow_rate[0, 0] == pytest.approx(m_dot_engine + tc.vent_rate[0, 0], rel=1e-9)
    assert tc.pressure[0, 0] > 0 and tc.refuel_volume_capped[0, 0] == 0.0

    # a repeated call with the same inputs restores the cached solve
    tc.pressure[:, :] = 0.0
    solve(tank, state)
    assert tc.pressure[0, 0] > 0


# ----------------------------------------------------------------------------------------------------------------------
#  Refuel branches
# ----------------------------------------------------------------------------------------------------------------------
def refuel_case(fill_fraction, target_fraction, rate):
    tank, state = lh2_case(chemical_power=0.0)
    tc          = state.conditions.energy.sources[tank.tag]
    tc.fuel_mass[:, 0]             = fill_fraction * tank.design_full_liquid_mass
    tc.refuel_mass_flow_rate[:, 0] = rate
    tc.refuel_target_mass[:, 0]    = target_fraction * tank.design_full_liquid_mass
    return tank, state, tc


def test_refuel_below_target_fills_without_cutoff():
    tank, state, tc = refuel_case(0.2, 0.9, 0.005)
    m_l0 = tc.fuel_mass[0, 0]
    tc   = solve(tank, state)
    assert np.all(tc.refuel_mass_flow_rate[:, 0] == 0.005)       # neither the target nor the volume limit is reached
    assert tc.fuel_mass[-1, 0] > m_l0
    assert tc.fuel_mass[-1, 0] < tank.design_full_liquid_mass
    assert tc.refuel_volume_capped[0, 0] == 0.0


def test_refuel_starting_at_volume_limit_holds():
    tank, state, tc = refuel_case(1.0, 2.0, 0.05)
    # liquid already fills net_volume, so no fuel is added though the mass target is not met
    tc.fuel_mass[:, 0] = 1.001 * tank.volume_properties.net_volume * \
        tank.fuel.cryogen_properties(np.atleast_1d(tc.fuel_temperature[0, 0]), "Density (kg/m3)", phase='liquid')[0]
    assert liquid_volume(tank, tc.fuel_mass[0, 0], tc.fuel_temperature[0, 0]) >= tank.volume_properties.net_volume
    tc = solve(tank, state)
    assert np.all(tc.refuel_mass_flow_rate[:, 0] == 0.0)
    assert tc.refuel_volume_capped[0, 0] == 1.0


# ----------------------------------------------------------------------------------------------------------------------
#  Python fallback when the compiled right-hand side needs it
# ----------------------------------------------------------------------------------------------------------------------
def test_python_fallback_matches_kernel(monkeypatch):
    tank, state = lh2_case()
    expected    = solve(tank, state)
    expected    = {field: np.array(expected[field], copy=True) for field in OUTPUTS}

    # every kernel call reports a heat-leak solve it cannot do, so the Python tank_state_rates runs instead
    rates = kernel.cryogenic_tank_rates
    monkeypatch.setattr(performance, 'cryogenic_tank_rates', lambda *args: (*rates(*args)[:2], 1))
    monkeypatch.setattr(performance, 'cryogenic_tank_node_diagnostics', lambda *args: None)
    tank, state = lh2_case()
    tc          = solve(tank, state)
    for field in OUTPUTS:
        np.testing.assert_allclose(tc[field][:, 0], expected[field][:, 0], rtol=1e-6, atol=1e-12, err_msg=field)


# ----------------------------------------------------------------------------------------------------------------------
#  Integration failures
# ----------------------------------------------------------------------------------------------------------------------
class FailedSolve:
    success = False
    message = 'forced failure'


@pytest.mark.parametrize("case", ['flight', 'fill', 'hold_at_limit'])
def test_failed_integration_raises(monkeypatch, case):
    monkeypatch.setattr(performance, 'solve_ivp', lambda *args, **kwargs: FailedSolve())
    if case == 'flight':
        tank, state = lh2_case()
    elif case == 'fill':
        tank, state, tc = refuel_case(0.2, 0.9, 0.005)
    else:
        tank, state, tc = refuel_case(1.0, 2.0, 0.05)
        tc.fuel_mass[:, 0] = 2.0 * tank.design_full_liquid_mass
    with pytest.raises(RuntimeError, match='failed to integrate'):
        solve(tank, state)


def test_failed_fill_or_hold_after_cutoff_raises(monkeypatch):
    # the first (event-detecting) fill succeeds; the spliced fill or hold solve then fails
    real_solve_ivp = performance.solve_ivp
    for failing_call in (2, 3):
        calls = {'n': 0}
        def solve_ivp(*args, **kwargs):
            calls['n'] += 1
            return FailedSolve() if calls['n'] == failing_call else real_solve_ivp(*args, **kwargs)
        monkeypatch.setattr(performance, 'solve_ivp', solve_ivp)
        tank, state, tc = refuel_case(0.2, 0.5, 0.2)
        with pytest.raises(RuntimeError, match='fill' if failing_call == 2 else 'hold'):
            solve(tank, state)
