# reformer_fuel_cell_test.py
#
# Created:  Aug 2026, RCAIDE Team

#----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import RCAIDE
from RCAIDE.Framework.Core import Units, Data

import numpy as np
import time

def main():
    ti = time.time()

    ctrl_pts = 1

    # ---- minimal state/segment scaffold (no full mission) ----
    state             = RCAIDE.Framework.Mission.Common.State()
    state.conditions  = RCAIDE.Framework.Mission.Common.Results()
    state.numerics.number_of_control_points = ctrl_pts

    segment       = Data()
    segment.state = state

    # ---- network, distributors ----
    network = RCAIDE.Framework.Networks.Fuel()

    fuel_line               = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    fuel_line.tag            = 'jet_a_line'
    fuel_line.working_fluid  = RCAIDE.Library.Attributes.Propellants.Jet_A()
    network.distributors.append(fuel_line)

    bus     = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    bus.tag = 'main_bus'
    network.distributors.append(bus)

    # ---- reformer / fuel cell composite ----
    reformer_fuel_cell                     = RCAIDE.Library.Components.Powertrain.Converters.Reformer_Fuel_Cell()
    reformer_fuel_cell.tag                 = 'apu_reformer_fuel_cell'
    reformer_fuel_cell.reformer            = RCAIDE.Library.Components.Powertrain.Converters.Reformer()
    reformer_fuel_cell.reformer.working_fluid = RCAIDE.Library.Attributes.Propellants.Jet_A()
    reformer_fuel_cell.fuel_cell           = RCAIDE.Library.Components.Powertrain.Converters.Generic_Fuel_Cell_Stack()
    reformer_fuel_cell.assigned_distributors = [[fuel_line.tag, bus.tag]]
    network.converters.append(reformer_fuel_cell)

    # ---- conditions ----
    reformer_fuel_cell.append_operating_conditions(segment)
    fuel_line.append_operating_conditions(segment)
    bus.append_operating_conditions(segment)

    # electrical demand on the bus that the composite must supply, and the
    # battery/fuel-cell split ratio (0 -> fuel cell supplies all of it)
    target_electrical_power = 50000.  # [W]
    state.conditions.energy.distributors[bus.tag].outputs.power.electrical[:,0] = target_electrical_power
    state.conditions.energy.battery_fuel_cell_power_split_ratio[bus.tag]        = 0. * state.ones_row(1)

    inputs, outputs, stored_results_flag, stored_converter_tag = reformer_fuel_cell.compute_performance(state,network)

    electrical_out = outputs.power.electrical[0,0]
    chemical_in    = inputs.power.chemical[0,0]
    H2_flow        = state.conditions.energy.converters[reformer_fuel_cell.fuel_cell.tag].H2_mass_flow_rate[0]
    fuel_flow      = state.conditions.energy.converters[reformer_fuel_cell.tag].fuel_mass_flow_rate[0,0]
    S_C            = state.conditions.energy.converters[reformer_fuel_cell.reformer.tag].steam_to_carbon_feed_ratio[0,0]

    print('Reformer_Fuel_Cell results:')
    print('  electrical_out [W]   :', electrical_out)
    print('  chemical_in    [W]   :', chemical_in)
    print('  H2_flow      [kg/s]  :', H2_flow)
    print('  fuel_flow    [kg/s]  :', fuel_flow)
    print('  S_C          [-]     :', S_C)

    # the composite must supply exactly the bus's demand (power_split_ratio=1, psi=0)
    assert np.abs(electrical_out - target_electrical_power) < 1e-6
    # a real hydrogen flow was produced and a real fuel flow was drawn to produce it
    assert H2_flow > 0
    assert fuel_flow > 0
    # conversion losses (reforming + fuel cell) mean less power comes out than chemical energy goes in
    assert chemical_in > electrical_out
    # the reformer's steam-to-carbon ratio is a pure function of its fixed design
    # ratio and fuel constants (scale-invariant in the fuel feed rate), so it must
    # be close to reformer_test.py's independently-validated truth value (small
    # difference is rounding in design_steam_to_fuel_volumetric_ratio's 4-sig-fig
    # default vs that test's full-precision implied ratio), confirming the solved
    # fuel feed rate was fed back through the forward model consistently
    assert np.abs(S_C - 3.4873653186988016) < 1e-3

    print('Reformer_Fuel_Cell test passed')

    elapsed_time = time.time() - ti
    print('Elapsed time (min): ', elapsed_time/60)
    return

if __name__ == '__main__':
    main()
