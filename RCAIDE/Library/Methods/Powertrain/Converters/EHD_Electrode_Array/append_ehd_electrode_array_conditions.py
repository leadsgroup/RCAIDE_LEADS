# RCAIDE/Library/Methods/Powertrain/Converters/EHD_Electrode_Array/append_ehd_electrode_array_conditions.py
#
# Created:  Oct 2026, RCAIDE EHD MVP

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Mission.Common import Conditions

# ----------------------------------------------------------------------------------------------------------------------
#  append_ehd_electrode_array_conditions
# ----------------------------------------------------------------------------------------------------------------------
def append_ehd_electrode_array_conditions(electrode_array, segment, energy_conditions):
    """
    Allocates the per-control-point results of an EHD electrode array (spec 3.3 step 10, group 6).

    Parameters
    ----------
    electrode_array : RCAIDE.Library.Components.Powertrain.Converters.EHD_Electrode_Array
    segment : RCAIDE.Framework.Mission.Segments.Segment
    energy_conditions : RCAIDE.Framework.Mission.Common.Conditions

    Returns
    -------
    None
        Arrays are created in energy_conditions.converters[electrode_array.tag]: relative_air_density,
        inception_field, inception_voltage, applied_voltage, normalized_voltage, dimensionless_current,
        unit_current, unit_thrust, unit_power, collector_reynolds_number, collector_drag_coefficient,
        unit_collector_drag, unit_wire_drag, sparkover_flag (all per unit, SI).
    """
    ones_row = segment.state.ones_row
    c = Conditions()
    for key in ('relative_air_density', 'inception_field', 'inception_voltage', 'applied_voltage',
                'normalized_voltage', 'dimensionless_current', 'unit_current', 'unit_thrust', 'unit_power',
                'collector_reynolds_number', 'collector_drag_coefficient', 'unit_collector_drag',
                'unit_wire_drag', 'sparkover_flag'):
        c[key] = 0. * ones_row(1)
    energy_conditions.converters[electrode_array.tag] = c
    return
