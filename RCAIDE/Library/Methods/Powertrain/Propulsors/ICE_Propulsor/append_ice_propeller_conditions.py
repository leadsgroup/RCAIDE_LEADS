# RCAIDE/Library/Methods/Powertrain/Propulsors/ICE_Propulsor/append_ice_propeller_conditions.py
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions 

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_ice_propeller_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_ice_propeller_conditions(propulsor,segment):  
    """
    Initializes the energy and noise condition containers for an Internal Combustion Engine (ICE) 
    propeller propulsion system. Sets up the basic state variables needed for propeller and 
    engine performance analysis.

    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Propulsors.ICE_Propeller
        The ICE propeller propulsion system
            - tag : str
                Identifier for the propulsor
    segment : RCAIDE.Framework.Mission.Segments.Segment
        The mission segment being analyzed
            - state : State
                Contains the flight condition state variables
                    - conditions : Conditions
                        Energy and noise conditions containers
                    - ones_row : function
                        Returns array of ones with specified size

    Returns
    -------
    None

    Notes
    -----
    Creates and initializes the following state variables:
        - throttle : float
            Engine power setting from 0 to 1
        - commanded_thrust_vector_angle : float
            Desired thrust angle [rad]
        - thrust : array_like
            Force vector [N]
        - power : float
            Power consumption [W]
        - moment : array_like
            Moment vector [N-m]
        - fuel_flow_rate : float
            Rate of fuel consumption [kg/s]
        - inputs/outputs : Conditions
            Container for additional operating parameters
    """
    
    ones_row    = segment.state.ones_row                  
    segment.state.conditions.energy[propulsor.tag]                               = Conditions()  
    segment.state.conditions.energy[propulsor.tag].throttle                      = 0. * ones_row(1)      
    segment.state.conditions.energy[propulsor.tag].commanded_thrust_vector_angle = 0. * ones_row(1)  
    segment.state.conditions.energy[propulsor.tag].thrust                        = 0. * ones_row(3) 
    segment.state.conditions.energy[propulsor.tag].power                         = 0. * ones_row(1) 
    segment.state.conditions.energy[propulsor.tag].moment                        = 0. * ones_row(3) 
    segment.state.conditions.energy[propulsor.tag].fuel_flow_rate                = 0. * ones_row(1)
    segment.state.conditions.energy[propulsor.tag].inputs                        = Conditions()
    segment.state.conditions.energy[propulsor.tag].outputs                       = Conditions() 
    segment.state.conditions.noise[propulsor.tag]                                = Conditions() 
    segment.state.conditions.noise[propulsor.tag].rotor                          = Conditions() 
                
    return 