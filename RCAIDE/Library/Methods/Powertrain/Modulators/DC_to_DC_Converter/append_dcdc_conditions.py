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
    energy_conditions.modulators[dcdc_converter.tag].inputs.Vin         = 0. * ones_row(1)  # [V] input DC voltage
    energy_conditions.modulators[dcdc_converter.tag].inputs.eta         = 0. * ones_row(1)  # [-] converter efficiency 
    energy_conditions.modulators[dcdc_converter.tag].inputs.Vout_target = 0. * ones_row(1)  # [V] desired output voltage 
    energy_conditions.modulators[dcdc_converter.tag].inputs.Rload       = 0. * ones_row(1)  # [ohm] DC load 
    energy_conditions.modulators[dcdc_converter.tag].inputs.Iout_set    = 0. * ones_row(1)  # [A] desired output current  
    energy_conditions.modulators[dcdc_converter.tag].outputs.Vout       = 0. * ones_row(1) 
    energy_conditions.modulators[dcdc_converter.tag].outputs.duty       = 0. * ones_row(1) 
    energy_conditions.modulators[dcdc_converter.tag].outputs.Iout       = 0. * ones_row(1) 
    energy_conditions.modulators[dcdc_converter.tag].outputs.Pin        = 0. * ones_row(1) 
    energy_conditions.modulators[dcdc_converter.tag].outputs.Pout       = 0. * ones_row(1) 
    energy_conditions.modulators[dcdc_converter.tag].outputs.Iin        = 0. * ones_row(1) 
    energy_conditions.modulators[dcdc_converter.tag].outputs.R_equiv    = 0. * ones_row(1) 
    return 