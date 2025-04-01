# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/append_ice_residual_and_unknown.py
# 
# Created:  Jun 2024, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import  Units

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_ice_residual_and_unknown
# ----------------------------------------------------------------------------------------------------------------------  
def append_ice_residual_and_unknown(propulsor,segment):
    """
    Initializes the propeller RPM unknown and torque matching residual for an Internal Combustion 
    Engine (ICE) propulsion system. Sets the initial RPM based on the propeller's design angular 
    velocity.

    Parameters
    ----------
    propulsor : RCAIDE.Core.Systems.Propulsors
        The ICE propulsion system
            - tag : str
                Identifier for the propulsor
            - propeller : Component
                The propeller component with design parameters
                    - cruise : Conditions
                        Contains design angular velocity
    segment : RCAIDE.Core.Analyses.Mission.Segments
        The mission segment being analyzed
            - state : State
                Contains solver unknowns and residuals
                    - unknowns : dict
                        Storage for solver variables
                    - residuals : Residuals
                        Network residuals container

    Returns
    -------
    None

    Notes
    -----
    The function sets up two key variables for the solver:
        1. Propeller RPM unknown - initialized from cruise design conditions
        2. Engine-propeller torque matching residual - initialized to zero

    **Definitions**

    'Torque Residual'
        Difference between engine and propeller torques that must be driven to zero
    """
    
    ones_row    = segment.state.ones_row                   
    propeller  = propulsor.propeller 
    segment.state.unknowns[propulsor.tag  + '_propeller_rpm'] = ones_row(1) * float(propeller.cruise.design_angular_velocity) /Units.rpm   
    segment.state.residuals.network[ propulsor.tag + '_rotor_engine_torque'] = 0. * ones_row(1)
    
    return 