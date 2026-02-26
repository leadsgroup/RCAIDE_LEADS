# RCAIDE/Library/Components/Powertrain/Converters/Pump.py
# 
# Created:  Jan 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Pump  import Pump
from RCAIDE.Library.Methods.Powertrain.Converters.Liquid_Hydrogen_Fuel_Cell_Pump import compute_pump_performance, append_pump_conditions

 
# ----------------------------------------------------------------------------------------------------------------------
# Pump
# ----------------------------------------------------------------------------------------------------------------------            
class Liquid_Hydrogen_Fuel_Cell_Pump(Pump):
    """ 
    """
    def __defaults__(self): 
        """
        Sets default values for the system attributes.
        """         
        self.tag                     = 'lh2_pump'
        self.fuel_cell_efficiency    = .7
        self.fuel_cell_flow_rate_multipier = 1/120e6 # Power*1/120MW = additional flow rate

     
    def compute_performance(self,state,fuel_line = None,bus = None):
        """
        Computes Turboelectric_Generator performance including power.
        """
        P_mech,P_elec,stored_results_flag,stored_propulsor_tag =  compute_pump_performance(self,state,fuel_line, bus)
        return P_mech,P_elec,stored_results_flag,stored_propulsor_tag 
    
    def append_operating_conditions(self,segment): 
        """
        Adds operating conditions for the avionics system to a mission segment.

        Parameters
        ----------
        segment : Data
            Mission segment to which conditions are being added
        bus : Data
            Electrical bus supplying power to the avionics
        """
        append_pump_conditions(self,segment) 
        return 