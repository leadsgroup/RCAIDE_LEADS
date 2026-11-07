# RCAIDE/Library/Components/Powertrain/Systems/Miscellaneous.py
# 
# Created:  Jan 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems import Systems
 
# ----------------------------------------------------------------------
# Water_Tank
# ----------------------------------------------------------------------
class Miscellaneous(Systems):
    """
    A class representing an onboard water tank used to store water for
    injection into the propulsion system or for cabin/auxiliary use.

    Attributes
    ----------
    tag : str
        Unique identifier for the component. Default is 'miscellaneous'.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Systems.Systems
    """
    def __defaults__(self):
        """
        Sets default values for the water tank attributes.
        """
        self.tag        = 'miscellaneous'