# RCAIDE/Library/Methods/Powertrain/Converters/Ram/append_ram_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_ram_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_ram_conditions(ram, segment, propulsor_conditions): 
    """
    Initializes and appends ram conditions data structure to the propulsor conditions dictionary.
    
    Parameters
    ----------
    ram : Ram
        The ram component for which conditions are being initialized.
    segment : Segment
        The mission segment in which the ram is operating.
    propulsor_conditions : dict
        Dictionary containing conditions for all propulsion components.
    
    Returns
    -------
    None
        This function modifies the propulsor_conditions dictionary in-place.
    
    Notes
    -----
    This function creates empty Conditions objects for the ram's inputs and outputs
    within the propulsor_conditions dictionary. These conditions will be populated during
    the mission analysis process.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Ram.compute_ram_performance
    """
    propulsor_conditions[ram.tag]                              = Conditions() 
    propulsor_conditions[ram.tag].inputs                       = Conditions() 
    propulsor_conditions[ram.tag].outputs                      = Conditions() 
    return 