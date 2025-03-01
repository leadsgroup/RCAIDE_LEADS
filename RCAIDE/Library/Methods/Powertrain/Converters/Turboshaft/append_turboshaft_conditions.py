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
def append_turboshaft_conditions(turboshaft,segment,converter):
    '''
    MATTEO
    
    '''
    ones_row    = segment.state.ones_row                 
    
    converter_results                              = segment.state.conditions.energy[converter.tag] 
    converter_results[turboshaft.tag]              = Conditions()

    converter_results[turboshaft.tag].throttle                      = 0. * ones_row(1)     
    converter_results[turboshaft.tag].commanded_thrust_vector_angle = 0. * ones_row(1)   
    converter_results[turboshaft.tag].shaft_power                   = 0. * ones_row(1)
    converter_results[turboshaft.tag].fuel_flow_rate                = 0. * ones_row(1)
    converter_results[turboshaft.tag].angular_velocity              = turboshaft.design_angular_velocity * ones_row(1) 
    converter_results[turboshaft.tag].inputs                        = Conditions()
    converter_results[turboshaft.tag].outputs                       = Conditions()
 
    turboshaft_conditions      = converter_results[turboshaft.tag]
    for tag, item in  turboshaft.items(): 
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment,turboshaft_conditions) 
            for sub_tag, sub_item in  item.items(): 
                if issubclass(type(sub_item), RCAIDE.Library.Components.Component):
                    item_conditions = turboshaft_conditions[item.tag] 
                    sub_item.append_operating_conditions(segment,item_conditions) 
    return 