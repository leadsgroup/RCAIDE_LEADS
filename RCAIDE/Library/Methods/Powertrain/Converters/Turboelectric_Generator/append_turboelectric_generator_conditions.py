# RCAIDE/Library/Methods/Powertrain/Converters/Turboelectric_Generator/append_turboelectric_generator_conditions.py 
# 
# Created:  Feb 2025, M. Clarke  
import RCAIDE
from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_turboelectric_generator_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turboelectric_generator_conditions(turboelectric_generator,segment,energy_conditions):  

    ones_row    = segment.state.ones_row   
 
    energy_conditions[turboelectric_generator.tag] = Conditions() 
    energy_conditions[turboelectric_generator.tag].throttle                                   = 0. * ones_row(1)     
    energy_conditions[turboelectric_generator.tag].commanded_thrust_vector_angle              = 0. * ones_row(1)   
    energy_conditions[turboelectric_generator.tag].power                                      = 0. * ones_row(1)
    energy_conditions[turboelectric_generator.tag].fuel_flow_rate                             = 0. * ones_row(1)
    energy_conditions[turboelectric_generator.tag].inputs                                     = Conditions()
    energy_conditions[turboelectric_generator.tag].outputs                                    = Conditions() 
  
    turboelectric_generator_conditions      = segment.state.conditions.energy[turboelectric_generator.tag]
    
    turboshaft = turboelectric_generator.turboshaft
    generator = turboelectric_generator.generator
    turboshaft.append_operating_conditions(segment,turboelectric_generator_conditions)
    generator.append_operating_conditions(segment,turboelectric_generator_conditions)
    return 