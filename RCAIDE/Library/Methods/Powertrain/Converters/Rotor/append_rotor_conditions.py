# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/append_rotor_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_rotor_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_rotor_conditions(rotor,segment,energy_conditions,noise_conditions): 
    ones_row    = segment.state.ones_row 
    energy_conditions.converters[rotor.tag]                               = Conditions()   
    energy_conditions.converters[rotor.tag].orientation                   = 0. * ones_row(3) 
    energy_conditions.converters[rotor.tag].design_flag                   = False
    energy_conditions.converters[rotor.tag].optimize_blade_pitch          = True
    energy_conditions.converters[rotor.tag].commanded_thrust_vector_angle = 0. * ones_row(1) 
    energy_conditions.converters[rotor.tag].blade_pitch_command           = ones_row(1) * rotor.blade_pitch_command 
    energy_conditions.converters[rotor.tag].torque                        = 0. * ones_row(1)
    energy_conditions.converters[rotor.tag].motor_torque                  = 0. * ones_row(1)
    energy_conditions.converters[rotor.tag].throttle                      = ones_row(1)
    energy_conditions.converters[rotor.tag].thrust                        = 0. * ones_row(1)
    energy_conditions.converters[rotor.tag].rpm                           = 0. * ones_row(1)
    energy_conditions.converters[rotor.tag].omega                         = 0. * ones_row(1)
    energy_conditions.converters[rotor.tag].disc_loading                  = 0. * ones_row(1)                 
    energy_conditions.converters[rotor.tag].power_loading                 = 0. * ones_row(1)
    energy_conditions.converters[rotor.tag].tip_mach                      = 0. * ones_row(1)
    energy_conditions.converters[rotor.tag].efficiency                    = 0. * ones_row(1)
    energy_conditions.converters[rotor.tag].figure_of_merit               = 0. * ones_row(1)
    energy_conditions.converters[rotor.tag].power_coefficient             = 0. * ones_row(1) 
    noise_conditions.converters[rotor.tag]                                = Conditions() 
    return 
