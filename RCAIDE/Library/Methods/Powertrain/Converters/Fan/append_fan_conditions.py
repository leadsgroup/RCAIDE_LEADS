# RCAIDE/Library/Methods/Powertrain/Converters/Fan/append_fan_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_fan_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_fan_conditions(fan, segment, propulsor_conditions): 
    """
    Initializes and appends fan conditions to the propulsor conditions dictionary.
    
    Parameters
    ----------
    fan : Fan
        The fan component for which conditions are being initialized.
    segment : Segment
        The mission segment in which the fan is operating.
    propulsor_conditions : dict
        Dictionary containing conditions for all propulsion components.
    
    Returns
    -------
    None
        This function modifies the propulsor_conditions dictionary in-place.
    
    Notes
    -----
    This function creates empty Conditions objects for the fan's inputs and outputs
    within the propulsor_conditions dictionary. These conditions will be populated during
    the mission analysis process.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Fan.compute_fan_performance
    """
    propulsor_conditions[fan.tag]                              = Conditions() 
    propulsor_conditions[fan.tag].inputs                       = Conditions() 
    propulsor_conditions[fan.tag].outputs                      = Conditions() 
    return 