# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/append_electric_ducted_fan_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
 # RCAIDE imports  
from RCAIDE.Framework.Mission.Common                             import Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append electric ducted fan network conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_electric_ducted_fan_conditions(propulsor,segment):
    """
    Appends and initializes the state conditions for an electric ducted fan propulsion system.
    
    Parameters
    ----------
    propulsor : RCAIDE.Components.Propulsors.Electric_Ducted_Fan
        The electric ducted fan propulsion system object
    segment : RCAIDE.Framework.Mission.Segments.Segment
        The mission segment to which the conditions will be appended
        
    Returns
    -------
    None
        
    Notes
    -----
    This function initializes various performance parameters for the electric ducted fan
    system. All values are initialized to zero and updated during mission analysis.
    
    The following conditions are initialized:
        - throttle
        - commanded thrust vector angle
        - thrust vector (3D)
        - power
        - moment vector (3D)
    
    See Also
    --------
    RCAIDE.Library.Components.Propulsors.Electric_Ducted_Fan
    RCAIDE.Framework.Mission.Common.Conditions
    """
    
    ones_row    = segment.state.ones_row
                
    segment.state.conditions.energy[propulsor.tag]                               = Conditions()  
    segment.state.conditions.energy[propulsor.tag].throttle                      = 0. * ones_row(1)      
    segment.state.conditions.energy[propulsor.tag].commanded_thrust_vector_angle = 0. * ones_row(1)  
    segment.state.conditions.energy[propulsor.tag].thrust                        = 0. * ones_row(3) 
    segment.state.conditions.energy[propulsor.tag].power                         = 0. * ones_row(1) 
    segment.state.conditions.energy[propulsor.tag].moment                        = 0. * ones_row(3)         
    return