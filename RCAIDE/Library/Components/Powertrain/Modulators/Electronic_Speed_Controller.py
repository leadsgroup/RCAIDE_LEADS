# RCAIDE/Library/Components/Powertrain/Modulators/Electronic_Speed_Controller.py
#  
# Created:  Mar 2024, M. Clarke 
# Modified: Oct 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
from .Modulator                             import Modulator
from RCAIDE.Library.Methods.Powertrain.Modulators.Electronic_Speed_Controller.append_esc_conditions   import append_esc_conditions 
from RCAIDE.Library.Methods.Powertrain.Modulators.Electronic_Speed_Controller.compute_esc_performance import compute_esc_performance
 
# ----------------------------------------------------------------------------------------------------------------------
#  Electronic Speed Controller Class
# ---------------------------------------------------------------------------------------------------------------------- 
class Electronic_Speed_Controller(Modulator):
    """
    Class for modeling electronic speed controllers in electric propulsion systems
    
    Attributes
    ----------
    tag : str
        Identifier for the ESC (default: 'electronic_speed_controller')
        
    efficiency : float
        Power conversion efficiency of the ESC (default: 0.0)

    Notes
    -----
    The Electronic Speed Controller (ESC) regulates power delivery to electric motors,
    controlling motor speed and torque. The efficiency attribute accounts for power
    losses during voltage and current modulation.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.Motor
        Electric motor components controlled by the ESC
    """
    
    def __defaults__(self):
        """
        Sets default values for ESC attributes
        
        Notes
        -----
        Initializes the ESC with a default tag and zero efficiency. The efficiency
        should be set to an appropriate value based on the specific ESC being modeled.
        """         
 
        self.tag                   = 'electronic_speed_controller'  
        self.bus_voltage           = None

    def append_operating_conditions(self,segment): 
        append_esc_conditions(self,segment)
        return 
    
    def compute_performance(self,state):

        inputs, outputs, stored_results_flag, stored_modulator_tag = compute_esc_performance(self,state)
        return inputs, outputs, stored_results_flag, stored_modulator_tag