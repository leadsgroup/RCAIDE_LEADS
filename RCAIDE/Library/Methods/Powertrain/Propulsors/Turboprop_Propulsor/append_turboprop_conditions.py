# RCAIDE/Library/Methods/Powertrain/Propulsors/Turboprop_Propulsor/append_turboprop_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_turboprop_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turboprop_conditions(turboprop,segment):  
    ones_row    = segment.state.ones_row                  
    segment.state.conditions.energy[turboprop.tag]                               = Conditions()  
    segment.state.conditions.energy[turboprop.tag].throttle                      = 0. * ones_row(1)     
    segment.state.conditions.energy[turboprop.tag].commanded_thrust_vector_angle = 0. * ones_row(1)   
    segment.state.conditions.energy[turboprop.tag].power                         = 0. * ones_row(1) 
    segment.state.conditions.energy[turboprop.tag].fuel_flow_rate                = 0. * ones_row(1)
    segment.state.conditions.energy[turboprop.tag].inputs                        = Conditions()
    segment.state.conditions.energy[turboprop.tag].outputs                       = Conditions() 
    segment.state.conditions.noise[turboprop.tag]                                = Conditions() 
    segment.state.conditions.noise[turboprop.tag].turboprop                      = Conditions() 
    segment.state.conditions.noise[turboprop.tag].turboprop.core_nozzle          = Conditions()   
    return 