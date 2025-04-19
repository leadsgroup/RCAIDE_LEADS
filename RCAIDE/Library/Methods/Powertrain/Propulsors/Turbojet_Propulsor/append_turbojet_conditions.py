# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbojet_Propulsor/append_turbojet_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_turbojet_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turbojet_conditions(turbojet,segment):  
    ones_row    = segment.state.ones_row                  
    segment.state.conditions.energy[turbojet.tag]                               = Conditions()  
    segment.state.conditions.energy[turbojet.tag].throttle                      = 0. * ones_row(1)     
    segment.state.conditions.energy[turbojet.tag].commanded_thrust_vector_angle = 0. * ones_row(1)    
    segment.state.conditions.energy[turbojet.tag].thrust                        = 0. * ones_row(3) 
    segment.state.conditions.energy[turbojet.tag].power                         = 0. * ones_row(1) 
    segment.state.conditions.energy[turbojet.tag].moment                        = 0. * ones_row(3) 
    segment.state.conditions.energy[turbojet.tag].fuel_flow_rate                = 0. * ones_row(1)
    segment.state.conditions.energy[turbojet.tag].inputs                        = Conditions()
    segment.state.conditions.energy[turbojet.tag].outputs                       = Conditions() 
    segment.state.conditions.noise[turbojet.tag]                                = Conditions() 
    segment.state.conditions.noise[turbojet.tag].turbojet                       = Conditions() 
    segment.state.conditions.noise[turbojet.tag].turbojet.core_nozzle           = Conditions() 
    return 