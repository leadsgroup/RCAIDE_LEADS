# VnV/Verification/powertrain/test_compute_battery_properties.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                      import Units
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common import compute_battery_properties
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def test_module_energy_uses_nominal_voltage():
    # rated energy = cells x capacity x nominal voltage (not maximum voltage)
    module                                    = RCAIDE.Library.Components.Powertrain.Sources.Batteries.Modules.Lithium_Ion_NMC()
    module.electrical_configuration.series    = 10
    module.electrical_configuration.parallel  = 20
    module.geometric_configuration.normal_count   = 10
    module.geometric_configuration.parallel_count = 20
    pack = RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack()
    pack.append_module(module)
    compute_battery_properties(pack, None)
    expected = 10 * 20 * module.cell.nominal_capacity * module.cell.nominal_voltage * Units.Wh
    assert np.isclose(pack.maximum_energy, expected, rtol=1e-12)
    assert module.cell.maximum_voltage != module.cell.nominal_voltage


def test_auto_sized_pack_delivers_design_capacity():
    # pack sized from design_capacity holds that energy, to the rounding up of whole cells
    module                               = RCAIDE.Library.Components.Powertrain.Sources.Batteries.Modules.Lithium_Ion_NMC()
    module.BMS_additional_weight_factor  = 1.0
    module.cell.nominal_voltage          = 4.5
    module.cell.maximum_voltage          = 6.0
    module.cell.watt_hour_rating         = module.cell.nominal_capacity * module.cell.nominal_voltage
    module.cell.specific_energy          = 638. * Units.Wh / Units.kg
    module.cell.mass                     = module.cell.watt_hour_rating * Units.Wh / module.cell.specific_energy
    pack                                 = RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack()
    pack.design_capacity                 = 700. * Units.kWh
    pack.design_power                    = 200E3
    pack.design_voltage                  = 270.
    pack.append_module(module)
    compute_battery_properties(pack, None)
    assert 1.0 <= pack.maximum_energy / pack.design_capacity < 1.01


def main():
    test_module_energy_uses_nominal_voltage()
    test_auto_sized_pack_delivers_design_capacity()
    return


if __name__ == '__main__':
    main()
