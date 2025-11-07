# RCAIDE/Library/Methods/Powertrain/Modulators/DC_to_DC_Converter/append_dcdc_conditions.py
# 
#
# Created:  Sep 2025, M. Guidotti

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_dcdc_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_dcdc_conditions(dcdc_converter,segment): 
    """
   
    """
    ones_row                                                                                  = segment.state.ones_row 
    segment.state.conditions.energy.modulators[dcdc_converter.tag]                            = Conditions()
    segment.state.conditions.energy.modulators[dcdc_converter.tag].inputs                     = Conditions()
    segment.state.conditions.energy.modulators[dcdc_converter.tag].outputs                    = Conditions() 
    segment.state.conditions.energy.modulators[dcdc_converter.tag].inputs.voltage             = 0. * ones_row(1)  # [V] input DC voltage
    segment.state.conditions.energy.modulators[dcdc_converter.tag].inputs.efficiency          = 0. * ones_row(1)  # [-] converter efficiency 
    segment.state.conditions.energy.modulators[dcdc_converter.tag].inputs.voltage_target      = 0. * ones_row(1)  # [V] desired output voltage 
    segment.state.conditions.energy.modulators[dcdc_converter.tag].inputs.resistance          = 0. * ones_row(1)  # [ohm] DC load 
    segment.state.conditions.energy.modulators[dcdc_converter.tag].inputs.current             = 0. * ones_row(1)  # [A] desired output current  
    segment.state.conditions.energy.modulators[dcdc_converter.tag].outputs.voltage            = 0. * ones_row(1) 
    segment.state.conditions.energy.modulators[dcdc_converter.tag].outputs.duty_cycle         = 0. * ones_row(1) 
    segment.state.conditions.energy.modulators[dcdc_converter.tag].outputs.current            = 0. * ones_row(1) 
    segment.state.conditions.energy.modulators[dcdc_converter.tag].inputs.current             = 0. * ones_row(1) 
    segment.state.conditions.energy.modulators[dcdc_converter.tag].outputs.resistance         = 0. * ones_row(1) 
    segment.state.conditions.energy.modulators[dcdc_converter.tag].inputs.power               = Conditions() 
    segment.state.conditions.energy.modulators[dcdc_converter.tag].inputs.power.propulsive    = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[dcdc_converter.tag].inputs.power.mechanical    = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[dcdc_converter.tag].inputs.power.electrical    = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[dcdc_converter.tag].inputs.power.chemical      = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[dcdc_converter.tag].inputs.power.pneumatic     = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[dcdc_converter.tag].inputs.power.hydraulic     = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[dcdc_converter.tag].inputs.power.thermal       = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[dcdc_converter.tag].outputs.power              = Conditions() 
    segment.state.conditions.energy.modulators[dcdc_converter.tag].outputs.power.propulsive   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[dcdc_converter.tag].outputs.power.mechanical   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[dcdc_converter.tag].outputs.power.electrical   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[dcdc_converter.tag].outputs.power.chemical     = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[dcdc_converter.tag].outputs.power.pneumatic    = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[dcdc_converter.tag].outputs.power.hydraulic    = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[dcdc_converter.tag].outputs.power.thermal      = 0 * ones_row(1)
    return 