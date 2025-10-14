# RCAIDE/Library/Methods/Powertrain/Converters/Fan/append_fan_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_fan_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_fan_conditions(fan,segment): 
    """
    Initializes and appends fan conditions to the energy conditions dictionary.
    
    Parameters
    ----------
    fan : Fan
        The fan component for which conditions are being initialized.
    segment : Segment
        The mission segment in which the fan is operating.
    
    Returns
    -------
    None
        This function modifies the segment.state.conditions.energy dictionary in-place.
    
    Notes
    -----
    This function creates empty Conditions objects for the fan's inputs and outputs
    within the segment.state.conditions.energy dictionary. These conditions will be populated during
    the mission analysis process.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Fan.compute_fan_performance
    """
    ones_row    = segment.state.ones_row                  
    segment.state.conditions.energy.converters[fan.tag]                              = Conditions() 
    segment.state.conditions.energy.converters[fan.tag].inputs                       = Conditions() 
    segment.state.conditions.energy.converters[fan.tag].outputs                      = Conditions()
    segment.state.conditions.energy.converters[fan.tag].rpm                          = 0. * ones_row(1) 
    
    return 