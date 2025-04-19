# RCAIDE/Library/Methods/Powertrain/Propulsors/Constant_Speed_ICE_Propulsor/append_ice_cs_propeller_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_ice_cs_propeller_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_ice_cs_propeller_conditions(ice_cs_propeller,segment):  
    ones_row    = segment.state.ones_row                  
    segment.state.conditions.energy[ice_cs_propeller.tag]                               = Conditions()  
    segment.state.conditions.energy[ice_cs_propeller.tag].throttle                      = 0. * ones_row(1)      
    segment.state.conditions.energy[ice_cs_propeller.tag].commanded_thrust_vector_angle = 0. * ones_row(1)  
    segment.state.conditions.energy[ice_cs_propeller.tag].thrust                        = 0. * ones_row(3) 
    segment.state.conditions.energy[ice_cs_propeller.tag].power                         = 0. * ones_row(1) 
    segment.state.conditions.energy[ice_cs_propeller.tag].moment                        = 0. * ones_row(3) 
    segment.state.conditions.energy[ice_cs_propeller.tag].fuel_flow_rate                = 0. * ones_row(1)
    segment.state.conditions.energy[ice_cs_propeller.tag].rpm                           = segment.state.conditions.energy.rpm * ones_row(1)      
    segment.state.conditions.energy[ice_cs_propeller.tag].inputs                        = Conditions()
    segment.state.conditions.energy[ice_cs_propeller.tag].outputs                       = Conditions() 
    segment.state.conditions.noise[ice_cs_propeller.tag]                                = Conditions()  
    return 