# RCAIDE/Library/Components/Powertrain/Converters/Cryogenic_Pump.py
# 
# Created:  Jan 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from RCAIDE.Framework.Core.Units import Units

from .Pump  import Pump
from RCAIDE.Library.Methods.Powertrain.Converters.Cryogenic_Pump import compute_cryogenic_pump_performance, append_cryogenic_pump_conditions

 
# ----------------------------------------------------------------------------------------------------------------------
# Pump
# ----------------------------------------------------------------------------------------------------------------------            
class Cryogenic_Pump(Pump):
    """ 
    """
    def __defaults__(self): 
        """
        Sets default values for the system attributes.
        """         
        self.tag                           = 'cryogenic_pump'
        self.design_inlet_pressure         = 2 * Units.bar
        self.design_outlet_pressure        = 350 * Units.bar
        self.power_density                 = None
        self.turbine_efficiency            = None
        self.distributor_split             = None

     
    def compute_performance(self,state,network=None):
        """
        Computes cryogenic pump performance including shaft power and fuel consumption.
        """
        inputs,outputs,stored_results_flag,stored_converter_tag =  compute_cryogenic_pump_performance(self,state,network)
        return inputs,outputs,stored_results_flag,stored_converter_tag
    
    def append_operating_conditions(self,segment,network=None): 
        """
        Adds operating conditions for the cryogenic pump to a mission segment.

        Parameters
        ----------
        segment : Data
            Mission segment to which conditions are being added
        network : Data, optional
            Network supplying the cryogenic pump
        """
        append_cryogenic_pump_conditions(self,segment)
        return 