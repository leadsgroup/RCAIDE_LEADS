# RCAIDE/Library/Components/Powertrain/Modulators/Fuel_Selector.py
#  
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
from RCAIDE.Library.Components import Component
 
# ----------------------------------------------------------------------------------------------------------------------
#  Fuel_Selector
# ----------------------------------------------------------------------------------------------------------------------  
class Fuel_Selector(Component):
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

        self.tag              = 'fuel_selector'  
        self.efficiency       = 0.0       
     