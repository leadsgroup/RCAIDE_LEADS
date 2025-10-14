# RCAIDE/Library/Methods/Powertrain/Converters/Engine/append_engine_conditions.py
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_engine_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_engine_conditions(engine, segment): 
    """
    Initializes and appends engine operating conditions of the propulsor conditions data structure.
    
    Parameters
    ----------
    engine : RCAIDE.Library.Components.Propulsors
        Engine system instance for which conditions are being initialized
    segment : RCAIDE.Framework.Mission.Segments.Segment
        Mission segment instance containing flight conditions
        
    Returns
    -------
    None
        
    Notes
    -----
    This function creates a nested structure of Conditions objects to store engine
    inputs and outputs during mission analysis. The conditions are stored under
    the engine's unique tag identifier.
    """
    # unpack 
    ones_row          = segment.state.ones_row
    segment.state.conditions.energy.converters[engine.tag]                      = Conditions() 
    segment.state.conditions.energy.converters[engine.tag].inputs               = Conditions()
    segment.state.conditions.energy.converters[engine.tag].outputs              = Conditions()
    segment.state.conditions.energy.converters[engine.tag].torque               = 0 * ones_row(1)   
    segment.state.conditions.energy.converters[engine.tag].omega                = engine.rated_speed * ones_row(1)   
    
    return 
