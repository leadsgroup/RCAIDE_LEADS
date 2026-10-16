# VnV/Verification/powertrain/reformer_fuel_cell_test.py
#
# Created:  Aug 2026, RCAIDE Team
# Modified: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units, Data
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def solve_unit(fuel, electrical_demand=50000.):
    """Reformer_Fuel_Cell on a fuel line and bus, supplying electrical_demand [W]."""
    state            = RCAIDE.Framework.Mission.Common.State()
    state.conditions = RCAIDE.Framework.Mission.Common.Results()
    state.numerics.number_of_control_points = 1
    segment          = Data()
    segment.state    = state

    network                 = RCAIDE.Framework.Networks.Fuel()
    fuel_line               = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    fuel_line.working_fluid = fuel
    network.distributors.append(fuel_line)
    bus                     = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    network.distributors.append(bus)

    unit                        = RCAIDE.Library.Components.Powertrain.Converters.Reformer_Fuel_Cell()
    unit.working_fluid          = fuel
    unit.assigned_distributors  = [[fuel_line.tag, bus.tag]]
    network.converters.append(unit)

    unit.append_operating_conditions(segment)
    fuel_line.append_operating_conditions(segment)
    bus.append_operating_conditions(segment)
    state.conditions.energy.distributors[bus.tag].outputs.power.electrical[:, 0] = electrical_demand
    state.conditions.energy.battery_fuel_cell_power_split_ratio[bus.tag]        = 0. * state.ones_row(1)
    unit.compute_performance(state, network)
    return unit, state.conditions.energy.converters[unit.tag]

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def test_system_fuel_flow_is_demand_over_efficiency_and_lhv():
    # fuel drawn = electrical demand / (system efficiency x LHV), for any fuel
    Propellants = RCAIDE.Library.Attributes.Propellants
    for fuel in (Propellants.Jet_A(), Propellants.Liquid_Natural_Gas(), Propellants.Liquid_Petroleum_Gas()):
        unit, c = solve_unit(fuel)
        assert np.isclose(c.outputs.power.electrical[0, 0], 50000., rtol=1e-12)
        assert np.isclose(c.fuel_mass_flow_rate[0, 0], 50000. / (unit.system_efficiency * fuel.lower_heating_value), rtol=1e-12)


def test_system_defaults_are_nasa_raser_values():
    # Mak and Meier, NASA/CR-2007-214461/VOL1: 0.46 cruise efficiency (Table 27); 805.4 lb for 185 kW (Tables 27-28)
    unit = RCAIDE.Library.Components.Powertrain.Converters.Reformer_Fuel_Cell()
    assert unit.system_efficiency == 0.46
    assert np.isclose(185E3 / unit.system_specific_power, 805.4 * Units.lb, rtol=1e-12)


def main():
    test_system_fuel_flow_is_demand_over_efficiency_and_lhv()
    test_system_defaults_are_nasa_raser_values()
    return


if __name__ == '__main__':
    main()
