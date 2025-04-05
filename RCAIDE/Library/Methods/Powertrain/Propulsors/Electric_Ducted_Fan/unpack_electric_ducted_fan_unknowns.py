# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/unpack_electric_ducted_fan_unknowns.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  unpack electric ducted_fan network unknowns 
# ----------------------------------------------------------------------------------------------------------------------  

def unpack_electric_ducted_fan_unknowns(propulsor,segment):
    """Unpack electric ducted fan unknowns and assigns them to the specfic
    compoment each interation of the mission solver
    """
    motor      = propulsor.motor  
    motor_conditions = segment.state.conditions.energy.converters[motor.tag]
    motor_conditions.inputs.current = segment.state.unknowns[propulsor.tag + '_motor_current'] 
    return 