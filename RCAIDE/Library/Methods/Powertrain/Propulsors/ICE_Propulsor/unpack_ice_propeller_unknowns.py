# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/unpack_ice_propeller_unknowns.py
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  unpack ice propeller network unknowns 
# ----------------------------------------------------------------------------------------------------------------------  

def unpack_ice_propeller_unknowns(propulsor,segment):  
    """
    Unpacks the propeller RPM unknown from the solver state into the engine conditions.

    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Propulsors.ICE_Propeller
        The ICE propulsion system
            - tag : str
                Identifier for the propulsor
            - engine : Component
                The engine component that receives the RPM value
    segment : RCAIDE.Framework.Mission.Segments.Segment
        The mission segment being analyzed
            - state : State
                Contains solver unknowns and energy conditions
                    - unknowns : dict
                        Contains the RPM estimate
                    - conditions : Conditions
                        Energy conditions to be updated

    Returns
    -------
    None

    Notes
    -----
    This function transfers the solver's estimate of the rotational speed to the energy
    conditions where it will be used to compute engine and propeller performance.
    """
    
    engine            = propulsor.engine 
    segment.state.conditions.energy[propulsor.tag][engine.tag].rpm = segment.state.unknowns[propulsor.tag + '_propeller_rpm'] 
    return 