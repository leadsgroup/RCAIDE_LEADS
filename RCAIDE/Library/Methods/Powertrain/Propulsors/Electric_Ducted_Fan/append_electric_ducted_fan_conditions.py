# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/append_electric_ducted_fan_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Mission.Common                             import Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append electric ducted fan network conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_electric_ducted_fan_conditions(propulsor,segment):
    '''
    MATTEO
    
    '''
    # unpack 
    ones_row          = segment.state.ones_row
    energy_conditions = segment.state.conditions.energy
    noise_conditions  = segment.state.conditions.noise
    
    # add propulsor conditions 
    energy_conditions.propulsors[propulsor.tag]                               = Conditions()  
    energy_conditions.propulsors[propulsor.tag].throttle                      = 0. * ones_row(1)      
    energy_conditions.propulsors[propulsor.tag].commanded_thrust_vector_angle = 0. * ones_row(1)  
    energy_conditions.propulsors[propulsor.tag].thrust                        = 0. * ones_row(3) 
    energy_conditions.propulsors[propulsor.tag].power                         = 0. * ones_row(1) 
    energy_conditions.propulsors[propulsor.tag].moment                        = 0. * ones_row(3)

    # parse propulsor for comoonent and append      
    for tag, item in  propulsor.items(): 
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment,energy_conditions) 
            for sub_tag, sub_item in  item.items(): 
                if issubclass(type(sub_item), RCAIDE.Library.Components.Component):  
                    sub_item.append_operating_conditions(segment,energy_conditions)          
    return