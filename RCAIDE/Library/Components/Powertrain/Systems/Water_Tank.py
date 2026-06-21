# RCAIDE/Library/Components/Powertrain/Systems/Water_Tank.py
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
class Water_Tank(Systems):
    """
    A class representing an onboard water tank used to store water for
    injection into the propulsion system or for cabin/auxiliary use.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Systems.Systems
    """
    def __defaults__(self):
        self.tag        = 'water_tank'