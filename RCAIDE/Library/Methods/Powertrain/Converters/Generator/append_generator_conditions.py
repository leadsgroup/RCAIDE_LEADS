# RCAIDE/Library/Methods/Powertrain/Converters/Generator/append_generator_conditions.py
# 
# Created:  Feb 2025, M. Guidotti 
from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_generator_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_generator_conditions(generator,segment,conditions):  
    ones_row                                             = segment.state.ones_row                  
    conditions.converters[generator.tag]                 = Conditions()
    conditions.converters[generator.tag].inputs          = Conditions()
    conditions.converters[generator.tag].inputs.torque   = 0. * ones_row(1) 
    conditions.converters[generator.tag].inputs.power    = 0. * ones_row(1)
    conditions.converters[generator.tag].inputs.omega    = 0. * ones_row(1)
    conditions.converters[generator.tag].outputs         = Conditions()
    conditions.converters[generator.tag].outputs.current = 0. * ones_row(1) 
    conditions.converters[generator.tag].outputs.voltage = 0. * ones_row(1) 
    
    return 

