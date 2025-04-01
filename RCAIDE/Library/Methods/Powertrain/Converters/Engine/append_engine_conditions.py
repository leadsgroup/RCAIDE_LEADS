# RCAIDE/Library/Methods/Powertrain/Converters/Engine/append_engine_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_engine_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_engine_conditions(engine,segment,propulsor_conditions): 
    """
    Initializes and appends engine operating conditions to the propulsor conditions data structure.
    
    Parameters
    ----------
    engine : RCAIDE.Library.Components.Propulsors
        Engine system instance for which conditions are being initialized
    segment : RCAIDE.Framework.Mission.Segments.Segment
        Mission segment instance containing flight conditions
    propulsor_conditions : dict
        variable onto which engine operating conditions are appended
        
    Returns
    -------
    None
        
    Notes
    -----
    This function creates a nested structure of Conditions objects to store engine
    inputs and outputs during mission analysis. The conditions are stored under
    the engine's unique tag identifier.
    
    **Major Assumptions**
        * Each engine has a unique tag identifier
    """
    
    propulsor_conditions[engine.tag]                      = Conditions() 
    propulsor_conditions[engine.tag].inputs               = Conditions()
    propulsor_conditions[engine.tag].outputs              = Conditions()      
    return 
