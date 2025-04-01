# RCAIDE/Library/Methods/Powertrain/Propulsors/Turboprop_Propulsor/append_turboprop_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_turboprop_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turboprop_conditions(turboprop,segment):  
    """
    Initializes and appends necessary conditions data structures for turboprop analysis to a mission segment.
    
    Parameters
    ----------
    turboprop : Turboprop
        Turboprop engine object containing component definitions and settings
    segment : Segment
        Mission segment object to which conditions will be appended
    
    Returns
    -------
    None
        Modifies segment.state.conditions in-place by adding:
            - energy[turboprop.tag] : Conditions
                - throttle : ndarray
                    Throttle setting [-]
                - commanded_thrust_vector_angle : ndarray
                    Commanded thrust vector angle [rad]
                - power : ndarray
                    Engine shaft power output [W]
                - fuel_flow_rate : ndarray
                    Fuel mass flow rate [kg/s]
                - inputs : Conditions
                    Input parameters for analysis
                - outputs : Conditions
                    Computed output parameters
            - noise[turboprop.tag] : Conditions
                - turboprop : Conditions
                    Noise parameters for turboprop components
                - core_nozzle : Conditions
                    Noise parameters for core nozzle
    
    Notes
    -----
    This function initializes arrays with zeros using the segment's ones_row method to ensure
    proper dimensioning. The conditions data structures are used throughout the turboprop
    analysis process to store intermediate and final results.
    
    **Major Assumptions**
        * Initial conditions are set to zero
        * Array dimensions match segment discretization
        * Core nozzle noise is tracked separately from propeller noise
        * Power-based propulsion system (rather than pure thrust-based)
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop_Propulsor.compute_turboprop_performance
    """
    
    ones_row    = segment.state.ones_row                  
    segment.state.conditions.energy[turboprop.tag]                               = Conditions()  
    segment.state.conditions.energy[turboprop.tag].throttle                      = 0. * ones_row(1)     
    segment.state.conditions.energy[turboprop.tag].commanded_thrust_vector_angle = 0. * ones_row(1)   
    segment.state.conditions.energy[turboprop.tag].power                         = 0. * ones_row(1) 
    segment.state.conditions.energy[turboprop.tag].fuel_flow_rate                = 0. * ones_row(1)
    segment.state.conditions.energy[turboprop.tag].inputs                        = Conditions()
    segment.state.conditions.energy[turboprop.tag].outputs                       = Conditions() 
    segment.state.conditions.noise[turboprop.tag]                                = Conditions() 
    segment.state.conditions.noise[turboprop.tag].turboprop                      = Conditions() 
    segment.state.conditions.noise[turboprop.tag].turboprop.core_nozzle          = Conditions()   
    return 