# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/append_rotor_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_motor_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_rotor_conditions(rotor,segment,energy_conditions,noise_conditions): 
    ones_row    = segment.state.ones_row 
    energy_conditions[rotor.tag]                               = Conditions()   
    energy_conditions[rotor.tag].orientation                   = 0. * ones_row(3) 
    energy_conditions[rotor.tag].commanded_thrust_vector_angle = 0. * ones_row(1) 
    energy_conditions[rotor.tag].pitch_command                 = ones_row(1) * rotor.pitch_command
    energy_conditions[rotor.tag].torque                        = 0. * ones_row(1)
    energy_conditions[rotor.tag].throttle                      = ones_row(1)
    energy_conditions[rotor.tag].thrust                        = 0. * ones_row(1)
    energy_conditions[rotor.tag].rpm                           = 0. * ones_row(1)
    energy_conditions[rotor.tag].omega                         = 0. * ones_row(1)
    energy_conditions[rotor.tag].disc_loading                  = 0. * ones_row(1)                 
    energy_conditions[rotor.tag].power_loading                 = 0. * ones_row(1)
    energy_conditions[rotor.tag].tip_mach                      = 0. * ones_row(1)
    energy_conditions[rotor.tag].efficiency                    = 0. * ones_row(1)
    energy_conditions[rotor.tag].figure_of_merit               = 0. * ones_row(1)
    energy_conditions[rotor.tag].power_coefficient             = 0. * ones_row(1) 
    noise_conditions[rotor.tag]                                = Conditions() 
    return 
