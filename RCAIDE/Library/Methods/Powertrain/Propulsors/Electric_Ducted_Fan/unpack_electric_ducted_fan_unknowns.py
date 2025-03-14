# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/unpack_electric_ducted_fan_unknowns.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke   

# ---------------------------------------------------------------------------------------------------------------------- 
#  unpack electric ducted_fan network unknowns 
# ----------------------------------------------------------------------------------------------------------------------  

def unpack_electric_ducted_fan_unknowns(propulsor,segment): 
    motor         = propulsor.motor  
    motor_results = segment.state.conditions.energy.converters[motor.tag]
    motor_results.rotor_power_coefficient = segment.state.unknowns[propulsor.tag  + '_ducted_fan_cp'] 
    return 