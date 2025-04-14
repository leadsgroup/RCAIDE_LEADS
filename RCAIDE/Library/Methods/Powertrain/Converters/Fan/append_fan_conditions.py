# RCAIDE/Library/Methods/Powertrain/Converters/Fan/append_fan_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_fan_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_fan_conditions(fan,segment,energy_conditions): 
    """
    Initializes and appends fan conditions to the energy conditions dictionary.
    
    Parameters
    ----------
    fan : Fan
        The fan component for which conditions are being initialized.
    segment : Segment
        The mission segment in which the fan is operating.
    energy_conditions : dict
        Dictionary containing conditions for all propulsion components.
    
    Returns
    -------
    None
        This function modifies the energy_conditions dictionary in-place.
    
    Notes
    -----
    This function creates empty Conditions objects for the fan's inputs and outputs
    within the energy_conditions dictionary. These conditions will be populated during
    the mission analysis process.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Fan.compute_fan_performance
    """
    ones_row    = segment.state.ones_row                  
    energy_conditions.converters[fan.tag]                              = Conditions() 
    energy_conditions.converters[fan.tag].inputs                       = Conditions() 
    energy_conditions.converters[fan.tag].outputs                      = Conditions()
    energy_conditions.converters[fan.tag].rpm                          = 0. * ones_row(1) 
    
    return 