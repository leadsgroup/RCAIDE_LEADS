# VnV/Verification/analysis_weights/test_compute_propulsion_system_weight.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import pytest

import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Cryogenic.Common.compute_propulsion_system_weight import compute_fuel_system_weight


def test_fuel_cell_weights():
    fuel_cell_stack                      = RCAIDE.Library.Components.Powertrain.Converters.Generic_Fuel_Cell_Stack()
    fuel_cell_stack.mass_properties.mass = 150.0
    reformer_fuel_cell                   = RCAIDE.Library.Components.Powertrain.Converters.Reformer_Fuel_Cell()
    reformer_fuel_cell.design_power      = 370E3
    network                              = RCAIDE.Framework.Networks.Fuel()
    network.converters.append(fuel_cell_stack)
    network.converters.append(reformer_fuel_cell)
    vehicle                              = RCAIDE.Vehicle()
    vehicle.networks.append(network)

    WTANK, WLINE, WPUMP, WFC = compute_fuel_system_weight(vehicle, None)

    # the reformer system is sized at 185 kW per 805.4 lb (Mak and Meier, NASA/CR-2007-214461/VOL1, Case 2)
    reformer_mass = 2 * 805.4 * Units.lb
    assert reformer_fuel_cell.mass_properties.mass == pytest.approx(reformer_mass, rel=1e-12)
    assert WFC == pytest.approx(150.0 + reformer_mass, rel=1e-12)
    assert WTANK == WLINE == WPUMP == 0
