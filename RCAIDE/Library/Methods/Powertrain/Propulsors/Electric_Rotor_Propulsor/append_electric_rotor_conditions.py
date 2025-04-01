# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/append_electric_rotor_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
 # RCAIDE imports  
from RCAIDE.Framework.Mission.Common                      import Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append electric rotor network conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_electric_rotor_conditions(propulsor,segment): 
    """
    Initializes the energy and noise condition containers for an electric rotor propulsor system.
    Sets up the basic state variables needed for rotor performance analysis.

    Parameters
    ----------
    propulsor : RCAIDE.Core.Systems.Propulsors
        The electric rotor propulsion system
            - tag : str
                Identifier for the propulsor
    segment : RCAIDE.Framework.Mission.Segments.Segment
        The mission segment being analyzed
            - state : State
                Contains the flight condition state variables

    Returns
    -------
    None

    Notes
    -----
    Creates and initializes the following state variables:
        - throttle : float
            Power setting from 0 to 1
        - commanded_thrust_vector_angle : float
            Desired thrust angle [rad]
        - thrust : array_like
            Force vector [N]
        - power : float
            Power consumption [W]
        - moment : array_like
            Moment vector [N-m]
    """
    
    ones_row    = segment.state.ones_row
                
    segment.state.conditions.energy[propulsor.tag]                               = Conditions()  
    segment.state.conditions.energy[propulsor.tag].throttle                      = 0. * ones_row(1)      
    segment.state.conditions.energy[propulsor.tag].commanded_thrust_vector_angle = 0. * ones_row(1)  
    segment.state.conditions.energy[propulsor.tag].thrust                        = 0. * ones_row(3) 
    segment.state.conditions.energy[propulsor.tag].power                         = 0. * ones_row(1) 
    segment.state.conditions.energy[propulsor.tag].moment                        = 0. * ones_row(3)  
    segment.state.conditions.noise[propulsor.tag]                                = Conditions()  
    return
