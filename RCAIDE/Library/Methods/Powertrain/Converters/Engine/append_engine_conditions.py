# RCAIDE/Library/Methods/Powertrain/Converters/Engine/append_engine_conditions.py
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_engine_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_engine_conditions(engine,segment,energy_conditions,noise_conditions=None): 
    """
    Initializes and appends engine operating conditions to the propulsor conditions data structure.
    
    Parameters
    ----------
    engine : RCAIDE.Library.Components.Propulsors
        Engine system instance for which conditions are being initialized
    segment : RCAIDE.Framework.Mission.Segments.Segment
        Mission segment instance containing flight conditions
    energy_conditions : dict
        variable onto which engine operating conditions are appended
    noise_conditions : dict, optional
        variable onto which engine noise conditions are appended. Unused for now.
        
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
    # unpack 
    ones_row          = segment.state.ones_row
    
    energy_conditions.converters[engine.tag]                      = Conditions() 
    energy_conditions.converters[engine.tag].inputs               = Conditions()
    energy_conditions.converters[engine.tag].outputs              = Conditions()
    energy_conditions.converters[engine.tag].omega                = engine.rated_speed * ones_row(1)   
    
    return 
