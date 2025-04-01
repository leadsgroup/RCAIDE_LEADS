# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/pack_electric_ducted_fan_residuals.py 
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  pack electric ducted_fan network residuals
# ----------------------------------------------------------------------------------------------------------------------  

def pack_electric_ducted_fan_residuals(propulsor,segment):
    """
    Packs the torque matching residual between the motor and ducted fan into the network residuals.
    This residual ensures torque equilibrium in the mechanical coupling.

    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Propulsors.Electric_Ducted_Fan
        The electric ducted fan propulsion system
            - tag : str
                Identifier for the propulsor
            - motor : Component
                The motor component
            - ducted_fan : Component
                The ducted fan component
    segment : RCAIDE.Framework.Mission.Segments.Segment
        The mission segment being analyzed
            - state : State
                Contains the flight condition state variables and residuals

    Returns
    -------
    None

    Notes
    -----
    The residual is computed as the difference between motor torque and ducted fan torque.
    A converged solution will have this residual approach zero, indicating torque balance
    between the motor and ducted fan.

    **Major Assumptions**
        * Direct mechanical coupling between motor and ducted fan
        * No losses in the mechanical transmission

    """
    motor         = propulsor.motor
    ducted_fan    = propulsor.ducted_fan 
    q_motor       = segment.state.conditions.energy[propulsor.tag][motor.tag].torque
    q_ducted_fan  = segment.state.conditions.energy[propulsor.tag][ducted_fan.tag].torque 
    segment.state.residuals.network[propulsor.tag  + '_ducted_fan_motor_torque'] = q_motor - q_ducted_fan
    return 
