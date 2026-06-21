# RCAIDE/Library/Components/Powertrain/Converters/Pump.py
# 
# Created: Sep. 2025  M. Guidotti

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Components.Powertrain.Converters import Converter
from RCAIDE.Library.Methods.Powertrain.Converters.Pump.append_pump_conditions import append_pump_conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Pump.compute_pump_performance import compute_pump_performance

# ----------------------------------------------------------------------
#  Pump
# ----------------------------------------------------------------------
class Pump(Converter):
    """
    """

    def __defaults__(self):
        """
        
        """
        self.tag        = 'Pump'
        self.delta_p    = 0.0

        return
    
    def append_operating_conditions(self, segment):
        """Attach motor operating conditions to the segment's energy conditions."""
        append_pump_conditions(self, segment)
        return
    
    def compute_performance(self,state):

        inputs, outputs, stored_results_flag, stored_converter_tag =  compute_pump_performance(self,state)
        return inputs, outputs, stored_results_flag, stored_converter_tag
    

    # def __defaults__(self):
    #     """
    #     Sets default values for the pump attributes.
    #     """
    #     self.tag        = 'Pump'
    #     self.efficiency = 1.0
    #     return
   
    # def compute_power_consumed(pressure_differential, density, mass_flow_rate, efficiency):
    #     """
    #     Calculates the power consumed by the pump.

    #     Parameters
    #     ----------
    #     pressure_differential : float
    #         Pressure rise across the pump
            
    #     density : float
    #         Coolant density
            
    #     mass_flow_rate : float
    #         Mass flow rate through the pump
            
    #     efficiency : float
    #         Overall pump efficiency

    #     Returns
    #     -------
    #     float
    #         Power consumed by the pump

    #     Notes
    #     -----
    #     Uses the standard pump power equation:
    #     Power = (mass_flow_rate * pressure_differential) / (density * efficiency)
    #     """
    #     return mass_flow_rate * pressure_differential / (density * efficiency)