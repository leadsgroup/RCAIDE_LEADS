# RCAIDE/Library/Methods/Powertrain/Propulsors/Turboshaft_Propulsor/append_turboshaft_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_turboshaft_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turboshaft_conditions(turboshaft,segment):  
    ones_row    = segment.state.ones_row                  
    segment.state.conditions.energy[turboshaft.tag]                               = Conditions()  
    segment.state.conditions.energy[turboshaft.tag].throttle                      = 0. * ones_row(1)     
    segment.state.conditions.energy[turboshaft.tag].commanded_thrust_vector_angle = 0. * ones_row(1)   
    segment.state.conditions.energy[turboshaft.tag].power                         = 0. * ones_row(1)
    segment.state.conditions.energy[turboshaft.tag].fuel_flow_rate                = 0. * ones_row(1)
    segment.state.conditions.energy[turboshaft.tag].inputs                        = Conditions()
    segment.state.conditions.energy[turboshaft.tag].outputs                       = Conditions() 
    segment.state.conditions.noise[turboshaft.tag]                                = Conditions() 
    segment.state.conditions.noise[turboshaft.tag].turboshaft                     = Conditions() 
    segment.state.conditions.noise[turboshaft.tag].turboshaft.core_nozzle         = Conditions()   
    return 