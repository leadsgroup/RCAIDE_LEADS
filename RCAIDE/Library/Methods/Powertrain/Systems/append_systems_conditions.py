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
    segment.state.conditions.energy.systems[system.tag].inputs                        = Conditions()
    segment.state.conditions.energy.systems[system.tag].inputs.current                = 0. * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].inputs.power                  = Conditions()
    segment.state.conditions.energy.systems[system.tag].inputs.power.propulsive       = 0. * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].inputs.power.mechanical       = 0. * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].inputs.power.electrical       = 0. * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].inputs.power.chemical         = 0. * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].inputs.power.pneumatic        = 0. * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].inputs.power.hydraulic        = 0. * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].inputs.power.thermal          = 0. * ones_row(1)  
    segment.state.conditions.energy.systems[system.tag].outputs                       = Conditions() 
    segment.state.conditions.energy.systems[system.tag].outputs.power                 = Conditions()
    segment.state.conditions.energy.systems[system.tag].outputs.current               = 0. * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].outputs.power.propulsive      = 0. * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].outputs.power.mechanical      = 0. * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].outputs.power.electrical      = 0. * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].outputs.power.chemical        = 0. * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].outputs.power.pneumatic       = 0. * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].outputs.power.hydraulic       = 0. * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].outputs.power.thermal         = 0. * ones_row(1) 
    
    return

def append_system_segment_conditions(system, segment): 
 
    system_conditions   = segment.state.conditions.energy.systems[system.tag]  
    system_conditions.inputs.power.electrical[:,0]             = 0.0
    system_conditions.inputs.power.thermal[:,0]                = 0.0
    system_conditions.inputs.power.hydraulic[:,0]              = 0.0
    system_conditions.inputs.power.propulsive[:,0]             = 0.0
    system_conditions.inputs.power.pneumatic[:,0]              = 0.0
    system_conditions.inputs.power.mechanical[:,0]             = 0.0
    system_conditions.inputs.power.chemical[:,0]               = 0.0 
    system_conditions.outputs.power.electrical[:,0]            = 0.0
    system_conditions.outputs.power.thermal[:,0]               = 0.0
    system_conditions.outputs.power.hydraulic[:,0]             = 0.0
    system_conditions.outputs.power.propulsive[:,0]            = 0.0
    system_conditions.outputs.power.pneumatic[:,0]             = 0.0
    system_conditions.outputs.power.mechanical[:,0]            = 0.0
    system_conditions.outputs.power.chemical[:,0]              = 0.0 
    return