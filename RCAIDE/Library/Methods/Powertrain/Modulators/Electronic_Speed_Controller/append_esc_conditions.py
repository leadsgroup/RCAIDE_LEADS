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
    segment.state.conditions.energy.modulators[esc.tag]                  = Conditions()
    segment.state.conditions.energy.modulators[esc.tag].inputs           = Conditions()
    segment.state.conditions.energy.modulators[esc.tag].outputs          = Conditions()
    segment.state.conditions.energy.modulators[esc.tag].throttle         = 0. * ones_row(1)  
    segment.state.conditions.energy.modulators[esc.tag].outputs.voltage  = 0. * ones_row(1)  
    segment.state.conditions.energy.modulators[esc.tag].inputs.voltage   = esc.bus_voltage * ones_row(1)   
    segment.state.conditions.energy.modulators[esc.tag].power              = Conditions() 
    segment.state.conditions.energy.modulators[esc.tag].power.propulsive   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[esc.tag].power.mechanical   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[esc.tag].power.electrical   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[esc.tag].power.chemical     = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[esc.tag].power.pneumatic    = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[esc.tag].power.hydraulic    = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[esc.tag].power.thermal      = 0 * ones_row(1)
    return 
