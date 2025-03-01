# RCAIDE/Library/Methods/Powertrain/Converters/Turboelectric_Generator/append_turboelectric_generator_conditions.py 
# 
# Created:  Feb 2025, M. Clarke  
import RCAIDE
from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_turboelectric_generator_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turboelectric_generator_conditions(turboelectric_generator,segment):  

    ones_row    = segment.state.ones_row   
 
    segment.state.conditions.energy[turboelectric_generator.tag] = Conditions() 
    segment.state.conditions.energy[turboelectric_generator.tag].throttle                                   = 0. * ones_row(1)     
    segment.state.conditions.energy[turboelectric_generator.tag].commanded_thrust_vector_angle              = 0. * ones_row(1)   
    segment.state.conditions.energy[turboelectric_generator.tag].power                                      = 0. * ones_row(1)
    segment.state.conditions.energy[turboelectric_generator.tag].fuel_flow_rate                             = 0. * ones_row(1)
    segment.state.conditions.energy[turboelectric_generator.tag].inputs                                     = Conditions()
    segment.state.conditions.energy[turboelectric_generator.tag].outputs                                    = Conditions() 
  
    turboelectric_generator_conditions      = segment.state.conditions.energy[turboelectric_generator.tag]
    for tag, item in  turboelectric_generator.items(): 
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment,turboelectric_generator)  
            for _, sub_item in  item.items(): 
                if issubclass(type(sub_item), RCAIDE.Library.Components.Component):
                    item_conditions = turboelectric_generator_conditions[item.tag] 
                    sub_item.append_operating_conditions(segment,item_conditions) 
    return 