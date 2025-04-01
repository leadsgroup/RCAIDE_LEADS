# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/pack_ice_propeller_residuals.py
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  pack ice propeller residuals
# ----------------------------------------------------------------------------------------------------------------------  

def pack_ice_propeller_residuals(propulsor,segment):  
    """
    Packs the torque matching residual between the engine and propeller into the network residuals.
    This residual ensures mechanical equilibrium in the engine-propeller coupling.

    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Propulsors.ICE_Propeller
        The ICE propulsion system
            - tag : str
                Identifier for the propulsor
            - engine : Component
                The engine component
            - propeller : Component
                The propeller component
    segment : RCAIDE.Framework.Mission.Segments.Segment
        The mission segment being analyzed
            - state : State
                Contains the flight condition state variables and residuals
                    - conditions : Conditions
                        Energy conditions containing torque values
                    - residuals : Residuals
                        Network residuals container

    Returns
    -------
    None

    Notes
    -----
    The residual is computed as the difference between engine torque and propeller torque.
    A converged solution will have this residual approach zero, indicating torque balance
    between the engine and propeller.

    **Major Assumptions**
        * Direct mechanical coupling between engine and propeller
        * No losses in the mechanical transmission
        * Positive torque convention is consistent between engine and propeller
    """
    
    engine             = propulsor.engine
    propeller          = propulsor.propeller  
    q_engine           = segment.state.conditions.energy[propulsor.tag][engine.tag].torque
    q_prop             = segment.state.conditions.energy[propulsor.tag][propeller.tag].torque 
    segment.state.residuals.network[propulsor.tag + '_rotor_engine_torque'] = q_engine - q_prop 
    return 
