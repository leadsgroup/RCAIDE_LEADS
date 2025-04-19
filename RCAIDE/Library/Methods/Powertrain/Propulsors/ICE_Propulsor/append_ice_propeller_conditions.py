# RCAIDE/Library/Methods/Powertrain/Propulsors/ICE_Propulsor/append_ice_propeller_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions 

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_ice_propeller_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_ice_propeller_conditions(propulsor,segment):  
    ones_row    = segment.state.ones_row                  
    segment.state.conditions.energy[propulsor.tag]                               = Conditions()  
    segment.state.conditions.energy[propulsor.tag].throttle                      = 0. * ones_row(1)      
    segment.state.conditions.energy[propulsor.tag].commanded_thrust_vector_angle = 0. * ones_row(1)  
    segment.state.conditions.energy[propulsor.tag].thrust                        = 0. * ones_row(3) 
    segment.state.conditions.energy[propulsor.tag].power                         = 0. * ones_row(1) 
    segment.state.conditions.energy[propulsor.tag].moment                        = 0. * ones_row(3) 
    segment.state.conditions.energy[propulsor.tag].fuel_flow_rate                = 0. * ones_row(1)
    segment.state.conditions.energy[propulsor.tag].inputs                        = Conditions()
    segment.state.conditions.energy[propulsor.tag].outputs                       = Conditions() 
    segment.state.conditions.noise[propulsor.tag]                                = Conditions() 
    segment.state.conditions.noise[propulsor.tag].rotor                          = Conditions() 
                
    return 