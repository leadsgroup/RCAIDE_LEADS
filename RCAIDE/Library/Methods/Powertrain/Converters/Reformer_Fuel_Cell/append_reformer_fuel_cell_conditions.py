# RCAIDE/Library/Methods/Powertrain/Converters/Reformer_Fuel_Cell/append_reformer_fuel_cell_conditions.py
#
# Created:  Aug 2026, RCAIDE Team

from RCAIDE.Framework.Mission.Common     import   Conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Common.append_converter_power_conditions import append_converter_power_conditions

# ----------------------------------------------------------------------------------------------------------------------
#  append_reformer_fuel_cell_conditions
# ----------------------------------------------------------------------------------------------------------------------
def append_reformer_fuel_cell_conditions(reformer_fuel_cell,segment):
    """
    Initializes and appends operating conditions data structures for a reformer/fuel-cell
    composite to the energy conditions structure.

    Parameters
    ----------
    reformer_fuel_cell : RCAIDE.Library.Components.Powertrain.Converters.Reformer_Fuel_Cell
        The composite converter for which conditions are being appended
    segment : RCAIDE.Framework.Mission.Segments.Segment
        The mission segment being evaluated

    Returns
    -------
    None
        This function modifies the segment.state.conditions.energy object in-place

    Notes
    -----
    This function initializes the condition structure for the reformer/fuel-cell
    composite and its subcomponents (reformer and fuel_cell) with zero values, then
    calls the respective append_operating_conditions methods for each subcomponent.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Reformer_Fuel_Cell.compute_reformer_fuel_cell_performance
    """

    ones_row  = segment.state.ones_row
    rfc_conditions = append_converter_power_conditions(reformer_fuel_cell, segment)
    rfc_conditions.fuel_mass_flow_rate = 0. * ones_row(1)

    reformer  = reformer_fuel_cell.reformer
    fuel_cell = reformer_fuel_cell.fuel_cell
    reformer.append_operating_conditions(segment)
    fuel_cell.append_operating_conditions(segment)
    return
