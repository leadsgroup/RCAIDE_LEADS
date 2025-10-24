# RCAIDE/Library/Components/Powertrain/Modulators/DC_to_DC_Converter.py
#  
# Created:  Sep. 2025, M. Guidotti 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
from RCAIDE.Framework.Core           import Data
from .Modulator                      import Modulator
from RCAIDE.Library.Methods.Powertrain.Modulators.DC_to_DC_Converter.append_dcdc_conditions   import append_dcdc_conditions 
from RCAIDE.Library.Methods.Powertrain.Modulators.DC_to_DC_Converter.compute_dcdc_performance import compute_dcdc_performance
 
# ----------------------------------------------------------------------------------------------------------------------
#  DC_to_DC_Converter Class
# ---------------------------------------------------------------------------------------------------------------------- 
class DC_to_DC_Converter(Modulator):
    """
   
    """
    
    def __defaults__(self):
        """
       
        """         

        self.tag                   = 'dc_to_dc_converter'  
        self.bus_voltage           = None

    def append_operating_conditions(self,segment): 
        append_dcdc_conditions(self,segment)
        return 
    
    def compute_performance(self,state):

        inputs, outputs, stored_results_flag, stored_modulator_tag =  compute_dcdc_performance(self,state)
        return inputs, outputs, stored_results_flag, stored_modulator_tag