# RCAIDE/Library/Methods/Powertrain/Converters/Common/append_converter_power_conditions.py
#
# Created:  Sep 2026, RCAIDE Team

from RCAIDE.Framework.Mission.Common import Conditions

# ----------------------------------------------------------------------------------------------------------------------
#  append_converter_power_conditions
# ----------------------------------------------------------------------------------------------------------------------
def append_converter_power_conditions(converter, segment):
    """
    Creates and zero-initializes the input/output power-domain Conditions
    container (propulsive/mechanical/electrical/chemical/pneumatic/hydraulic/
    thermal) shared by every converter, in
    segment.state.conditions.energy.converters[converter.tag]. Callers add
    their own converter-specific fields onto the returned container.

    Returns
    -------
    converter_conditions : Conditions
        The newly created container, already registered under the
        converter's tag.
    """
    ones_row = segment.state.ones_row

    converter_conditions         = Conditions()
    converter_conditions.inputs  = Conditions()
    converter_conditions.outputs = Conditions()
    converter_conditions.inputs.power  = Conditions()
    converter_conditions.outputs.power = Conditions()
    for direction in (converter_conditions.inputs, converter_conditions.outputs):
        direction.power.propulsive = 0. * ones_row(1)
        direction.power.mechanical = 0. * ones_row(1)
        direction.power.electrical = 0. * ones_row(1)
        direction.power.chemical   = 0. * ones_row(1)
        direction.power.pneumatic  = 0. * ones_row(1)
        direction.power.hydraulic  = 0. * ones_row(1)
        direction.power.thermal    = 0. * ones_row(1)

    segment.state.conditions.energy.converters[converter.tag] = converter_conditions
    return converter_conditions
