# RCAIDE/Library/Methods/Powertrain/Systems/append_systems_conditions.py
# 
# Created:  Jun 2024, M. Clarke  
# Modified: Oct 2025, M. Guidotti

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_system_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_systems_conditions(system, segment):  
    """

    """
    ones_row                                                                       = segment.state.ones_row
    segment.state.conditions.energy.systems[system.tag]                            = Conditions()   
    segment.state.conditions.energy.systems[system.tag].inputs                     = Conditions()   
    segment.state.conditions.energy.systems[system.tag].inputs.power               = Conditions()  
    segment.state.conditions.energy.systems[system.tag].inputs.power.electrical    = system.power_draw * ones_row(1) 
    segment.state.conditions.energy.systems[system.tag].outputs                    = Conditions()  
    segment.state.conditions.energy.systems[system.tag].outputs.power              = Conditions()  
    segment.state.conditions.energy.systems[system.tag].outputs.power.electrical   = 0 * ones_row(1) 
    
    return

def append_system_segment_conditions(system, segment): 
    #energy_conditions  = segment.state.conditions.energy      
    #energy_conditions.systems[system.tag].inputs.power.electrical[:,0]    = 0.0    
    #energy_conditions.systems[system.tag].outputs.power.electrical[:,0]   = 0.0
    pass