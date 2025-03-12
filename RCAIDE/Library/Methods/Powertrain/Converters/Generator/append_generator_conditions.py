# RCAIDE/Library/Methods/Powertrain/Converters/Generator/append_generator_conditions.py
# 
# Created:  Feb 2025, M. Guidotti 
from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_generator_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_generator_conditions(generator,segment,conditions):  
    ones_row                                 = segment.state.ones_row                  
    conditions[generator.tag]                = Conditions()
    conditions[generator.tag].inputs         = Conditions()
    conditions[generator.tag].outputs        = Conditions()
    conditions[generator.tag].torque         = 0. * ones_row(1) 
    conditions[generator.tag].current        = 0. * ones_row(1) 
    conditions[generator.tag].voltage        = 0. * ones_row(1) 
    conditions[generator.tag].inputs.power   = 0. * ones_row(1)
    conditions[generator.tag].inputs.omega   = 0. * ones_row(1)
    
    return 

