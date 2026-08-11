# RCAIDE/Library/Components/Powertrain/Converters/Cryogenic_Pump.py
# 
# Created:  Jan 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
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
        self.tag                     = 'cryogenic_pump'
        self.fuel_cell_efficiency    = .7
        self.fuel_cell_flow_rate_multipier = 1/120e6 # Power*1/120MW = additional flow rate
        self.specific_power_density   = None

     
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
        append_cryogenic_pump_conditions(self,segment,network=network) 
        return 