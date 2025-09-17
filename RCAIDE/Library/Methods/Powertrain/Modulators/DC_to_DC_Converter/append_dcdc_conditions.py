# RCAIDE/Library/Methods/Powertrain/Modulators/DC_to_DC_Converter/append_dcdc_conditions.py
# 
#
# Created:  Sep 2025, M. Guidotti

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_dcdc_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_dcdc_conditions(dcdc_converter,segment,energy_conditions): 
    """
   
    """
    
    ones_row                                                            = segment.state.ones_row 
    energy_conditions.modulators[dcdc_converter.tag]                    = Conditions()
    energy_conditions.modulators[dcdc_converter.tag].inputs             = Conditions()
    energy_conditions.modulators[dcdc_converter.tag].outputs            = Conditions() 
    energy_conditions.modulators[dcdc_converter.tag].inputs.voltage     = 0. * ones_row(1)  # [V] input DC voltage
    energy_conditions.modulators[dcdc_converter.tag].inputs.efficiency  = 0. * ones_row(1)  # [-] converter efficiency 
    energy_conditions.modulators[dcdc_converter.tag].inputs.voltage_target = 0. * ones_row(1)  # [V] desired output voltage 
    energy_conditions.modulators[dcdc_converter.tag].inputs.resistance  = 0. * ones_row(1)  # [ohm] DC load 
    energy_conditions.modulators[dcdc_converter.tag].inputs.current     = 0. * ones_row(1)  # [A] desired output current  
    energy_conditions.modulators[dcdc_converter.tag].outputs.voltage    = 0. * ones_row(1) 
    energy_conditions.modulators[dcdc_converter.tag].outputs.duty_cycle = 0. * ones_row(1) 
    energy_conditions.modulators[dcdc_converter.tag].outputs.current    = 0. * ones_row(1) 
    energy_conditions.modulators[dcdc_converter.tag].inputs.power       = 0. * ones_row(1) 
    energy_conditions.modulators[dcdc_converter.tag].outputs.power      = 0. * ones_row(1) 
    energy_conditions.modulators[dcdc_converter.tag].inputs.current     = 0. * ones_row(1) 
    energy_conditions.modulators[dcdc_converter.tag].outputs.resistance = 0. * ones_row(1) 
    return 