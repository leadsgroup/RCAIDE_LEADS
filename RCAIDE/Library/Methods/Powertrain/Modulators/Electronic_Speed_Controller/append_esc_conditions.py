# RCAIDE/Library/Methods/Powertrain/Modulators/Electronic_Speed_Controller/append_motor_conditions.py
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_esc_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_esc_conditions(esc,segment): 
    """

    """
    
    ones_row    = segment.state.ones_row 
    segment.state.conditions.energy.modulators[esc.tag]                           = Conditions() 
    segment.state.conditions.energy.modulators[esc.tag].throttle                  = 0. * ones_row(1)  
    segment.state.conditions.energy.modulators[esc.tag].inputs                    = Conditions()
    segment.state.conditions.energy.modulators[esc.tag].inputs.power              = Conditions()     
    segment.state.conditions.energy.modulators[esc.tag].inputs.voltage            = 0 * ones_row(1) # esc.bus_voltage * ones_row(1)    
    segment.state.conditions.energy.modulators[esc.tag].inputs.power.propulsive   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[esc.tag].inputs.power.mechanical   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[esc.tag].inputs.power.electrical   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[esc.tag].inputs.power.chemical     = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[esc.tag].inputs.power.pneumatic    = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[esc.tag].inputs.power.hydraulic    = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[esc.tag].inputs.power.thermal      = 0 * ones_row(1) 
    segment.state.conditions.energy.modulators[esc.tag].outputs                   = Conditions()   
    segment.state.conditions.energy.modulators[esc.tag].outputs.voltage           = 0. * ones_row(1)  
    segment.state.conditions.energy.modulators[esc.tag].outputs.power             = Conditions()  
    segment.state.conditions.energy.modulators[esc.tag].outputs.power.propulsive  = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[esc.tag].outputs.power.mechanical  = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[esc.tag].outputs.power.electrical  = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[esc.tag].outputs.power.chemical    = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[esc.tag].outputs.power.pneumatic   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[esc.tag].outputs.power.hydraulic   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[esc.tag].outputs.power.thermal     = 0 * ones_row(1)
    
    
    
    return 

def append_esc_segment_conditions(self, segment):  
    energy_conditions  = segment.state.conditions.energy     
    energy_conditions.modulators[self.tag].inputs.power.propulsive[:,0]    = 0.0
    energy_conditions.modulators[self.tag].inputs.power.mechanical[:,0]    = 0.0 
    energy_conditions.modulators[self.tag].inputs.power.electrical[:,0]    = 0.0 
    energy_conditions.modulators[self.tag].inputs.power.chemical[:,0]      = 0.0
    energy_conditions.modulators[self.tag].inputs.power.pneumatic[:,0]     = 0.0 
    energy_conditions.modulators[self.tag].inputs.power.hydraulic[:,0]     = 0.0 
    energy_conditions.modulators[self.tag].inputs.power.thermal[:,0]       = 0.0 
    energy_conditions.modulators[self.tag].outputs.power.propulsive[:,0]   = 0.0
    energy_conditions.modulators[self.tag].outputs.power.mechanical[:,0]   = 0.0   
    energy_conditions.modulators[self.tag].outputs.power.electrical[:,0]   = 0.0 
    energy_conditions.modulators[self.tag].outputs.power.chemical[:,0]     = 0.0
    energy_conditions.modulators[self.tag].outputs.power.pneumatic[:,0]    = 0.0   
    energy_conditions.modulators[self.tag].outputs.power.hydraulic[:,0]    = 0.0   
    energy_conditions.modulators[self.tag].outputs.power.thermal[:,0]      = 0.0  