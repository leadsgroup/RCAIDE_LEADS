# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/pack_electric_rotor_residuals.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  pack electric rotor network residuals
# ----------------------------------------------------------------------------------------------------------------------  

def pack_electric_rotor_residuals(propulsor,segment): 
    """
    Packs the torque matching residual between the motor and rotor into the network residuals.
    This residual ensures mechanical equilibrium in the rotor-motor coupling.

    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Propulsors.Electric_Rotor
        The electric rotor propulsion system
            - tag : str
                Identifier for the propulsor
            - motor : Component
                The motor component
            - rotor : Component
                The rotor component
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
    The residual is computed as the difference between motor torque and rotor torque.
    A converged solution will have this residual approach zero, indicating torque balance
    between the motor and rotor. This difference is calculated and stored in this function.

    **Definitions**

    'Torque Residual'
        Difference between motor and rotor torques that must be driven to zero
        for mechanical equilibrium
    """
    
    propulsor_results   = segment.state.conditions.energy
    motor               = propulsor.motor
    rotor               = propulsor.rotor 
    q_motor             = propulsor_results[propulsor.tag][motor.tag].torque
    q_prop              = propulsor_results[propulsor.tag][rotor.tag].torque
    
    if rotor.fidelity == 'Blade_Element_Momentum_Theory_Helmholtz_Wake': 
        segment.state.residuals.network[ propulsor.tag + '_rotor_motor_torque'] = q_motor - q_prop 
    return 
