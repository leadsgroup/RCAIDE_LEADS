# RCAIDE/Library/Methods/Powertrain/Converters/Turboshaft/append_turboshaft_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
import RCAIDE
from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_turboshaft_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turboshaft_conditions(turboshaft,segment,energy_conditions):
    '''
    MATTEO
    
    '''
    ones_row    = segment.state.ones_row
    
    energy_conditions[turboshaft.tag]                               = Conditions() 
    energy_conditions[turboshaft.tag].throttle                      = 0. * ones_row(1)     
    energy_conditions[turboshaft.tag].commanded_thrust_vector_angle = 0. * ones_row(1)   
    energy_conditions[turboshaft.tag].power                         = 0. * ones_row(1)
    energy_conditions[turboshaft.tag].fuel_flow_rate                = 0. * ones_row(1)
    energy_conditions[turboshaft.tag].inputs                        = Conditions()
    energy_conditions[turboshaft.tag].outputs                       = Conditions()
 
    turboshaft_conditions      = energy_conditions[turboshaft.tag]
    for tag, item in  turboshaft.items(): 
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment,turboshaft_conditions) 
            for sub_tag, sub_item in  item.items(): 
                if issubclass(type(sub_item), RCAIDE.Library.Components.Component):
                    item_conditions = turboshaft_conditions[item.tag] 
                    sub_item.append_operating_conditions(segment,item_conditions) 
    return 