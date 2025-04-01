# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/unpack_electric_ducted_fan_unknowns.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  unpack electric ducted_fan network unknowns 
# ----------------------------------------------------------------------------------------------------------------------  

def unpack_electric_ducted_fan_unknowns(propulsor,segment): 
    """
    Unpacks the power coefficient unknown from the solver state into the motor conditions.

    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Propulsors.Electric_Ducted_Fan
        The electric ducted fan propulsion system
            - tag : str
                Identifier for the propulsor
            - motor : Component
                The motor component
    segment : RCAIDE.Framework.Mission.Segments.Segment
        The mission segment being analyzed
            - state : State
                Contains solver unknowns and energy conditions

    Returns
    -------
    None

    Notes
    -----
    The power coefficient is a key parameter in determining the motor's operating point
    and is adjusted by the solver until torque equilibrium is achieved between the
    motor and ducted fan.
    
    **Definitions**

    'Power Coefficient'
        Non-dimensional parameter representing the power absorbed by the ducted fan
    """
    
    results = segment.state.conditions.energy[propulsor.tag]
    motor   = propulsor.motor  
    results[motor.tag].rotor_power_coefficient = segment.state.unknowns[propulsor.tag  + '_ducted_fan_cp'] 
    return 