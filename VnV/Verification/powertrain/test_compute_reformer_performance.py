# VnV/Verification/powertrain/test_compute_reformer_performance.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                      import Units
from RCAIDE.Framework.Mission.Common                            import Conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Reformer      import compute_reformer_performance
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def reform(fuel, fuel_volume_flow_rate=4.5e-9):
    """Reformer outputs for a fuel feed at the reformer's design steam and air ratios."""
    reformer               = RCAIDE.Library.Components.Powertrain.Converters.Reformer()
    reformer.working_fluid = fuel
    conditions             = Conditions()
    conditions.inputs      = Conditions()
    conditions.outputs     = Conditions()
    conditions.fuel_volume_flow_rate  = np.array([[fuel_volume_flow_rate]])
    conditions.steam_volume_flow_rate = reformer.design_steam_to_fuel_volumetric_ratio * conditions.fuel_volume_flow_rate
    conditions.air_volume_flow_rate   = reformer.design_air_to_fuel_volumetric_ratio * conditions.fuel_volume_flow_rate
    for side in (conditions.inputs, conditions.outputs):
        side.power = Conditions()
    state = Conditions()
    state.ones_row = lambda n: np.ones((1, n))
    compute_reformer_performance(reformer, conditions, state)
    return reformer, conditions

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def test_hydrogen_energy_is_efficiency_times_fuel_energy():
    # for every fuel, regardless of its density, LHV of H2 out = efficiency x LHV of fuel in
    Propellants = RCAIDE.Library.Attributes.Propellants
    for fuel in (Propellants.Jet_A(), Propellants.Liquid_Natural_Gas(), Propellants.Liquid_Petroleum_Gas()):
        reformer, c = reform(fuel)
        fuel_power  = 4.5e-9 * fuel.density * fuel.lower_heating_value
        h2_power    = c.hydrogen_mass_flow_rate[0, 0] * reformer.hydrogen_lower_heating_value / reformer.hydrogen_molecular_weight * 1e6
        assert np.isclose(c.inputs.power.chemical[0, 0], fuel_power, rtol=1e-9)
        assert np.isclose(h2_power, reformer.efficiency * fuel_power, rtol=1e-9)
        assert np.isclose(c.reformer_efficiency[0, 0], 100 * reformer.efficiency)


def test_default_efficiency_is_halabi_methane_value():
    # Halabi et al. (2008): 78% thermal efficiency, 2.6 mol H2 per mol CH4
    reformer = RCAIDE.Library.Components.Powertrain.Converters.Reformer()
    assert reformer.efficiency == 0.78


def test_default_specific_power_is_nasa_raser_reformer():
    # NASA/CR-2007-214461 Table 8: 0.0343 kg per kW of fuel cell output, so 250 kW needs 8.6 kg of reformer
    reformer = RCAIDE.Library.Components.Powertrain.Converters.Reformer()
    assert np.isclose(250e3 / reformer.specific_power, 0.0343 * 250, rtol=1e-12)


def main():
    test_hydrogen_energy_is_efficiency_times_fuel_energy()
    test_default_efficiency_is_halabi_methane_value()
    test_default_specific_power_is_nasa_raser_reformer()
    return


if __name__ == '__main__':
    main()
