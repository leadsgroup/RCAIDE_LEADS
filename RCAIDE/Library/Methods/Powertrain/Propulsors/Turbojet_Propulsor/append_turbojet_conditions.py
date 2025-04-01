# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbojet_Propulsor/append_turbojet_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_turbojet_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turbojet_conditions(turbojet,segment):  
    """
    Initializes and appends necessary conditions data structures for turbojet analysis to a mission segment.
    
    Parameters
    ----------
    turbojet : Turbojet
        Turbojet engine object containing component definitions and settings
    segment : Segment
        Mission segment object to which conditions will be appended
    
    Returns
    -------
    None
        Modifies segment.state.conditions in-place by adding:
            - energy[turbojet.tag] : Conditions
                - throttle : ndarray
                    Throttle setting [-]
                - commanded_thrust_vector_angle : ndarray
                    Commanded thrust vector angle [rad]
                - thrust : ndarray
                    Three-dimensional thrust vector [N]
                - power : ndarray
                    Engine power output [W]
                - moment : ndarray
                    Three-dimensional moment vector [N*m]
                - fuel_flow_rate : ndarray
                    Fuel mass flow rate [kg/s]
                - inputs : Conditions
                    Input parameters for analysis
                - outputs : Conditions
                    Computed output parameters
            - noise[turbojet.tag] : Conditions
                - turbojet : Conditions
                    Noise parameters for turbojet
                - core_nozzle : Conditions
                    Noise parameters for core nozzle
    
    Notes
    -----
    This function initializes arrays with zeros using the segment's ones_row method to ensure
    proper dimensioning. The conditions data structures are used throughout the turbojet
    analysis process to store intermediate and final results.
    
    **Major Assumptions**
        * Initial conditions are set to zero
        * Array dimensions match segment discretization
        * Three-dimensional vectors for thrust and moment
    """
    
    ones_row    = segment.state.ones_row                  
    segment.state.conditions.energy[turbojet.tag]                               = Conditions()  
    segment.state.conditions.energy[turbojet.tag].throttle                      = 0. * ones_row(1)     
    segment.state.conditions.energy[turbojet.tag].commanded_thrust_vector_angle = 0. * ones_row(1)    
    segment.state.conditions.energy[turbojet.tag].thrust                        = 0. * ones_row(3) 
    segment.state.conditions.energy[turbojet.tag].power                         = 0. * ones_row(1) 
    segment.state.conditions.energy[turbojet.tag].moment                        = 0. * ones_row(3) 
    segment.state.conditions.energy[turbojet.tag].fuel_flow_rate                = 0. * ones_row(1)
    segment.state.conditions.energy[turbojet.tag].inputs                        = Conditions()
    segment.state.conditions.energy[turbojet.tag].outputs                       = Conditions() 
    segment.state.conditions.noise[turbojet.tag]                                = Conditions() 
    segment.state.conditions.noise[turbojet.tag].turbojet                       = Conditions() 
    segment.state.conditions.noise[turbojet.tag].turbojet.core_nozzle           = Conditions() 
    return 