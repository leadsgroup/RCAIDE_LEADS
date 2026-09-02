# RCAIDE/Library/Methods/Powertrain/Converters/Pump/append_pump_conditions.py
# 
# Created:  Sep. 2025, M. Clarke

from RCAIDE.Framework.Mission.Common     import   Conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Common.append_converter_power_conditions import append_converter_power_conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_pump_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_pump_conditions(pump,segment):

    """
    Initializes empty condition containers for pump analysis in the propulsion system.

    Parameters
    ----------
    pump : Pump
        The pump component being analyzed
    segment : Segment
        The mission segment being analyzed

    Returns
    -------
    None

    Notes
    -----
    This function creates empty Conditions containers that will be populated
    during pump performance calculations with input/output mechanical, pneumatic,
    and hydraulic power states.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Pump.compute_pump_performance
    """

    ones_row                    = segment.state.ones_row
    pump_conditions             = append_converter_power_conditions(pump, segment)
    pump_conditions.fuel_mass_flow_rate = 0. * ones_row(1)
    pump_conditions.inputs.p_in         = 0. * ones_row(1)
    pump_conditions.outputs.p_out       = 0. * ones_row(1)
    return