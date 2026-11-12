# RCAIDE/Library/Methods/Powertrain/Distributors/Coolant_Line/append_coolant_line_conditions.py
#
# Created: Jul 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Framework.Mission.Common     import   Conditions
from RCAIDE.Library.Methods.Powertrain.reset_power_conditions import reset_power_conditions

# ----------------------------------------------------------------------------------------------------------------------
#  METHODS
# ----------------------------------------------------------------------------------------------------------------------
def append_coolant_line_conditions(coolant_line,segment):
    """
    Appends conditions for the coolant line to the segment's energy conditions dictionary.

    Parameters
    ----------
    coolant_line : RCAIDE.Library.Components.Powertrain.Distributors.Coolant_Line

    Returns
    -------
    None
        This function modifies the segment.state.conditions.energy dictionary in-place.
    """
    ones_row = segment.state.ones_row
    segment.state.conditions.energy.distributors[coolant_line.tag]                           = Conditions()
    segment.state.conditions.energy.distributors[coolant_line.tag].inputs                    = Conditions()
    segment.state.conditions.energy.distributors[coolant_line.tag].inputs.power              = Conditions()
    segment.state.conditions.energy.distributors[coolant_line.tag].inputs.power.propulsive    = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[coolant_line.tag].inputs.power.mechanical    = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[coolant_line.tag].inputs.power.electrical    = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[coolant_line.tag].inputs.power.chemical      = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[coolant_line.tag].inputs.power.pneumatic     = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[coolant_line.tag].inputs.power.hydraulic     = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[coolant_line.tag].inputs.power.thermal       = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[coolant_line.tag].outputs                    = Conditions()
    segment.state.conditions.energy.distributors[coolant_line.tag].outputs.power              = Conditions()
    segment.state.conditions.energy.distributors[coolant_line.tag].outputs.power.propulsive   = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[coolant_line.tag].outputs.power.mechanical   = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[coolant_line.tag].outputs.power.electrical   = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[coolant_line.tag].outputs.power.chemical     = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[coolant_line.tag].outputs.power.pneumatic    = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[coolant_line.tag].outputs.power.hydraulic    = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[coolant_line.tag].outputs.power.thermal      = 0. * ones_row(1)

    return


def append_coolant_line_segment_conditions(coolant_line,segment):

    coolant_line_conditions = segment.state.conditions.energy.distributors[coolant_line.tag]
    reset_power_conditions(coolant_line_conditions)

    return
