# VnV/Verification/powertrain/test_fuel_tank_sizing_limits.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
import pytest
from scipy.optimize import brentq

import RCAIDE
from RCAIDE.Framework.Core import Container, Units
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Common.find_root import find_root, bracket_root
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Common.solve_insulation import solve_insulation
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_cylindrical_tank_volume import compute_cryogenic_cylindrical_tank_volume
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_tank_heater_design_power import compute_cryogenic_tank_heater_design_power
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Pressurized_Tank.compute_pressurized_cylindrical_tank_volume import compute_pressurized_cylindrical_tank_volume
from RCAIDE.Library.Methods.Powertrain.Propulsors.Common.extend_thrust_below_floor import extend_thrust_below_floor


def in_container(tank):
    tanks = Container()
    tanks.append(tank)
    return tanks


# ----------------------------------------------------------------------------------------------------------------------
#  Tanks that do not fit are removed
# ----------------------------------------------------------------------------------------------------------------------
def test_cryogenic_tank_shorter_than_diameter_is_removed():
    tank                    = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank()
    tank.tag                = 'cryo_tank'
    tank.diameters.external = 1.0
    tank.lengths.external   = 0.8
    tanks                   = in_container(tank)
    compute_cryogenic_cylindrical_tank_volume(tank, tanks)
    assert 'cryo_tank' not in tanks
    compute_cryogenic_cylindrical_tank_volume(tank)        # without a container the tank is only left unsized
    assert tank.wall_thickness is None


def pressurized_tank(length):
    tank                    = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Pressurized_Tank()
    tank.tag                = 'h2_tank'
    tank.design_pressure    = 700 * Units.bar
    tank.diameters.external = 1.0
    tank.lengths.external   = length
    return tank


def test_pressurized_tank_shorter_than_diameter_is_removed():
    tank  = pressurized_tank(0.8)
    tanks = in_container(tank)
    compute_pressurized_cylindrical_tank_volume(tank, tanks)
    assert 'h2_tank' not in tanks
    compute_pressurized_cylindrical_tank_volume(pressurized_tank(0.8))


def test_pressurized_tank_wall_thicker_than_cylinder_is_removed():
    # a 700 bar wall is thicker than half of a 0.1 mm cylindrical section
    tank  = pressurized_tank(1.0001)
    tanks = in_container(tank)
    compute_pressurized_cylindrical_tank_volume(tank, tanks)
    assert 'h2_tank' not in tanks
    compute_pressurized_cylindrical_tank_volume(pressurized_tank(1.0001))


def test_pressurized_tank_that_fits_is_kept():
    tank  = pressurized_tank(3.0)
    tanks = in_container(tank)
    compute_pressurized_cylindrical_tank_volume(tank, tanks)
    assert 'h2_tank' in tanks
    assert 0 < tank.volume_properties.net_volume < tank.volume_properties.gross_volume


# ----------------------------------------------------------------------------------------------------------------------
#  Root finding
# ----------------------------------------------------------------------------------------------------------------------
def test_find_root_matches_brentq():
    f = lambda x, a: x**2 - a
    assert find_root(f, 0.0, 3.0, args=(2.0,)) == pytest.approx(brentq(f, 0.0, 3.0, args=(2.0,)), abs=1e-9)


def test_find_root_tangent_zero_uses_minimizer():
    # no sign change: brentq fails and the minimizer finds the double root
    assert find_root(lambda x: (x - 1.5)**2, 0.0, 4.0, xtol=1e-9) == pytest.approx(1.5, abs=1e-4)


def test_find_root_non_finite_bracket_raises():
    with pytest.raises(ValueError, match="non-finite bracket"):
        find_root(lambda x: x, 0.0, np.nan)


def test_bracket_root():
    assert bracket_root(lambda x: x - 0.3, start=1e-3, factor=10) == (0.1, 1.0)
    assert bracket_root(lambda x: x + 1.0, start=1e-3, factor=10, limit=1.0) is None


def test_solve_insulation_with_and_without_bracket():
    # the residual's arguments are threaded through unchanged
    residual = lambda t, therm, tank, target: t - target
    assert solve_insulation(residual, None, None, 0.03) == pytest.approx(0.03, abs=1e-9)
    tangent  = lambda t, therm, tank, target: (t - target)**2
    assert solve_insulation(tangent, None, None, 0.03) == pytest.approx(0.03, abs=1e-3)


# ----------------------------------------------------------------------------------------------------------------------
#  Heater sizing, thrust below the matching floor, LNG saturation
# ----------------------------------------------------------------------------------------------------------------------
def test_heater_design_power_without_takeoff_power_is_none():
    tank                       = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank()
    tank.assigned_distributors = [['fuel_line']]
    network                    = RCAIDE.Framework.Networks.Fuel()
    assert compute_cryogenic_tank_heater_design_power(tank, network) is None


def test_extend_thrust_below_floor():
    assert extend_thrust_below_floor(1000.0, 5000.0, 0.1, 0.2) == pytest.approx(1000.0 * np.exp(-0.5))
    assert extend_thrust_below_floor(1000.0, 5000.0, 0.2, 0.2) == 1000.0
    # no span above the lower bound, or a non-increasing floor: hold the floor thrust
    assert extend_thrust_below_floor(1000.0, 5000.0, 0.1, 0.2, lower_bound=1000.0) == 1000.0
    assert extend_thrust_below_floor(1000.0, 0.0, 0.1, 0.2) == 1000.0


def test_lng_saturation_temperature_inverts_saturation_pressure():
    lng    = RCAIDE.Library.Attributes.Propellants.Liquid_Natural_Gas()
    T      = np.array([115.0, 130.0, 150.0])
    P_sat  = lng.cryogen_properties(T, "Pressure (MPa)", phase='liquid')
    np.testing.assert_allclose(lng.saturation_temperature(P_sat), T, rtol=1e-3)
