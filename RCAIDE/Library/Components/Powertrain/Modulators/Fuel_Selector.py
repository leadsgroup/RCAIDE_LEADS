# RCAIDE/Library/Components/Powertrain/Modulators/Fuel_Selector.py
#  
# Created:  Mar 2024, M. Clarke 
# Modified: Oct 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
from RCAIDE.Framework.Core import Data
from .Modulator            import Modulator
 
# ----------------------------------------------------------------------------------------------------------------------
#  Fuel_Selector
# ----------------------------------------------------------------------------------------------------------------------  
class Fuel_Selector(Modulator):
    """
    Class for managing fuel flow control between tanks and engines
    
    Attributes
    ----------
    tag : str
        Identifier for the fuel selector (default: 'fuel_selector')
        
    efficiency : float
        Fuel transfer efficiency through the selector (default: 0.0)

    Notes
    -----
    The Fuel Selector controls fuel routing between multiple fuel tanks and engines,
    managing fuel distribution and tank selection during aircraft operation.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks
        Fuel storage components
    RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line
        Fuel distribution components
    """
    
    def __defaults__(self):
        """
        Sets default values for fuel selector attributes
        
        Notes
        -----
        Initializes the selector with a default tag and zero efficiency. The efficiency
        should be set to an appropriate value based on the specific system configuration.
        """         

        self.tag                   = 'fuel_selector'  
        self.efficiency            = 0.0       
        
    def append_operating_conditions(self,segment): 
        append_fuel_selector_conditions(self,segment)
        return 

    def compute_performance(self,state):

        inputs, outputs, stored_results_flag, stored_modulator_tag =  compute_fuel_selector_performance(self,state)
        return inputs, outputs, stored_results_flag, stored_modulator_tag