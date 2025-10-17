# RCAIDE/Library/Components/Powertrain/Modulators/Inverter.py
#  
# Created:  Sep. 2025, M. Guidotti 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
from RCAIDE.Library.Components import Component 
from RCAIDE.Library.Methods.Powertrain.Modulators.Inverter.append_inverter_conditions   import append_inverter_conditions 
from RCAIDE.Library.Methods.Powertrain.Modulators.Inverter.compute_inverter_performance import compute_inverter_performance
 
# ----------------------------------------------------------------------------------------------------------------------
#  Inverter Class
# ---------------------------------------------------------------------------------------------------------------------- 
class Inverter(Component):
    """
    Class for modeling inverters in electric propulsion systems
    
    Attributes
    ----------
    tag : str
        Identifier for the inverter (default: 'inverter')
        
    efficiency : float
        Power conversion efficiency of the inverter (default: 0.0)

    Notes
    -----
    The Inverter

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.Motor
        Electric motor components controlled by the inverter
    """
    
    def __defaults__(self):
        """
        Sets default values for inverter attributes
        
        Notes
        -----
        Initializes the inverter with a default tag and zero efficiency. The efficiency
        should be set to an appropriate value based on the specific inverter being modeled.
        """         

        self.tag                   = 'inverter'  
        self.bus_voltage           = None
        self.assigned_distributors = []
        self.electrical_efficiency = 1.0
        self.mechanical_efficiency = 1.0
        self.hydraulic_efficiency  = 1.0
        self.thermal_efficiency    = 1.0

    def append_operating_conditions(self,segment): 
        """
        Append inverter operating conditions for a flight segment
        
        Parameters
        ----------
        segment : Segment
            Flight segment containing state conditions
        propulsor : Component
            Propulsor component associated with this inverter
            
        Notes
        -----
        Updates the segment conditions with inverter-specific parameters including
        power throughput and losses.
        """ 
        append_inverter_conditions(self,segment)
        return 
    
    def compute_performance(self,state):

        P_mech,P_elec, P_hydr, P_therm, m_dot_fuel, stored_results_flag,stored_modulator_tag =  compute_inverter_performance(self,state)
        return P_mech,P_elec, P_hydr, P_therm, m_dot_fuel, stored_results_flag,stored_modulator_tag