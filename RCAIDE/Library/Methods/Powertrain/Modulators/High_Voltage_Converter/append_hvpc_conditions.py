# RCAIDE/Library/Methods/Powertrain/Modulators/High_Voltage_Converter/append_hvpc_conditions.py
#
# Created:  Oct 2026, RCAIDE EHD MVP

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Mission.Common import Conditions

# ----------------------------------------------------------------------------------------------------------------------
#  append_hvpc_conditions
# ----------------------------------------------------------------------------------------------------------------------
def append_hvpc_conditions(high_voltage_converter, segment, energy_conditions):
    """
    Allocates high-voltage power converter results (spec 3.3 step 10).

    Parameters
    ----------
    high_voltage_converter : RCAIDE.Library.Components.Powertrain.Modulators.High_Voltage_Converter
    segment : RCAIDE.Framework.Mission.Segments.Segment
    energy_conditions : RCAIDE.Framework.Mission.Common.Conditions

    Returns
    -------
    None
        Creates energy_conditions.modulators[tag] with inputs.power, inputs.voltage, outputs.power,
        outputs.voltage, heat and input_voltage_flag, following the ESC pattern.
    """
    ones_row = segment.state.ones_row
    hvpc     = high_voltage_converter
    bus_V    = 0. if hvpc.bus_voltage is None else hvpc.bus_voltage
    energy_conditions.modulators[hvpc.tag]                    = Conditions()
    energy_conditions.modulators[hvpc.tag].inputs             = Conditions()
    energy_conditions.modulators[hvpc.tag].outputs            = Conditions()
    energy_conditions.modulators[hvpc.tag].inputs.voltage     = bus_V * ones_row(1)
    energy_conditions.modulators[hvpc.tag].inputs.power       = 0. * ones_row(1)
    energy_conditions.modulators[hvpc.tag].outputs.voltage    = 0. * ones_row(1)
    energy_conditions.modulators[hvpc.tag].outputs.power      = 0. * ones_row(1)
    energy_conditions.modulators[hvpc.tag].heat               = 0. * ones_row(1)
    energy_conditions.modulators[hvpc.tag].input_voltage_flag = 0. * ones_row(1)
    return
