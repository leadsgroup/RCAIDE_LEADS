# RCAIDE/Library/Components/Powertrain/Converters/Pump.py
# 
# Created: Sep. 2025  M. Guidotti

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
from RCAIDE.Framework.Core import Data, Units
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
        self.tag                      = 'Pump'
        self.casting_and_mount_factor = 2.0
        self.efficiency               = 0.85
        self.design_inlet_pressure    = 0
        self.design_outlet_pressure   = 3 * Units.bar
        self.power_density            = 15000
        self.turbine_efficiency       = 0.9
        self.distributor_split        = None

        return
    
    def append_operating_conditions(self, segment):
        """Attach pump operating conditions to the segment's energy conditions."""
        append_pump_conditions(self, segment)
        return

    def compute_performance(self,state,network=None):

        inputs, outputs, stored_results_flag, stored_converter_tag =  compute_pump_performance(self,state,network)
        return inputs, outputs, stored_results_flag, stored_converter_tag