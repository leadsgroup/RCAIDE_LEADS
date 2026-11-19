# VnV/Verification/powertrain/test_electrical_bus.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE


def test_reformer_fuel_cell_sets_bus_design_point():
    bus                                      = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    reformer_fuel_cell                       = RCAIDE.Library.Components.Powertrain.Converters.Reformer_Fuel_Cell()
    reformer_fuel_cell.design_voltage        = 540.0
    reformer_fuel_cell.design_power          = 185E3
    reformer_fuel_cell.assigned_distributors = [[bus.tag]]
    network                                  = RCAIDE.Framework.Networks.Fuel()
    network.distributors.append(bus)
    network.converters.append(reformer_fuel_cell)

    bus.initialize(network)

    assert bus.design_voltage == 540.0
    assert bus.design_power   == 185E3
    assert bus.mass_properties.mass > 0          # cable sized for the reformer's power
