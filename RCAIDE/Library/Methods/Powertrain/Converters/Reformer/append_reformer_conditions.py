# RCAIDE/Library/Methods/Powertrain/Converters/Reformer/append_reformer_conditions.py
#
# Created:  Jan 2025, M. Clarke, M. Guidotti

from RCAIDE.Framework.Mission.Common     import   Conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Common.append_converter_power_conditions import append_converter_power_conditions

# ----------------------------------------------------------------------------------------------------------------------
#  append_reformer_conditions
# ----------------------------------------------------------------------------------------------------------------------
def append_reformer_conditions(reformer, segment):
    """
    Initializes reformer operating conditions for a mission segment.

    Parameters
    ----------
    reformer : RCAIDE.Library.Components.Powertrain.Converters.Reformer
        Reformer component with the following attributes:
            - tag : str
                Identifier for the reformer
    segment : RCAIDE.Framework.Mission.Segments.Segment
        Mission segment with the following attributes:
            - state : Data
                Segment state
                    - ones_row : function
                        Function to create array of ones with specified length

    Returns
    -------
    None

    Notes
    -----
    This function initializes the necessary data structures for storing reformer
    operating conditions during a mission segment. It creates zero-initialized
    input/output power containers, plus the fuel, steam, and air feed rates that
    compute_reformer_performance requires -- these are set to their real values
    each iteration either by a standalone forward-mode evaluation, or by the
    enclosing Reformer_Fuel_Cell composite when the reformer is used to supply
    hydrogen to a fuel cell.

    The function initializes the following in segment.state.conditions.energy.converters[reformer.tag]:
        - inputs.power, outputs.power : Conditions
            Zero-initialized power channels (propulsive, mechanical, electrical,
            chemical, pneumatic, hydraulic, thermal). inputs.power.chemical is
            the Jet-A chemical power drawn in; outputs.power.chemical is the
            hydrogen-rich reformate chemical power produced -- these differ by
            the reformer's conversion losses, so they are tracked separately
            rather than shared between inputs and outputs.
        - fuel_volume_flow_rate, steam_volume_flow_rate, air_volume_flow_rate : ndarray
            Zero-initialized feed rates [m**3/s]
        - hydrogen_mass_flow_rate : ndarray
            Zero-initialized hydrogen production rate [kg/s]

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Reformer.compute_reformer_performance
    """
    ones_row             = segment.state.ones_row
    reformer_conditions  = append_converter_power_conditions(reformer, segment)

    reformer_conditions.fuel_volume_flow_rate   = 0. * ones_row(1)
    reformer_conditions.steam_volume_flow_rate  = 0. * ones_row(1)
    reformer_conditions.air_volume_flow_rate    = 0. * ones_row(1)
    reformer_conditions.hydrogen_mass_flow_rate = 0. * ones_row(1)
    return
