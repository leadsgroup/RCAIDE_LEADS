# RCAIDE/Library/Components/Powertrain/Systems/Flight_Attendants.py
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
class Flight_Attendants(Systems):
    """
    A class representing a flight attendant mass system.

    Attributes
    ----------
    tag : str
        Unique identifier for the component. Default is 'Flight_Attendants'.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Systems.Systems
    """
    def __defaults__(self):
        """
        Sets default values for the water tank attributes.
        """
        self.tag        = 'flight_attendants'