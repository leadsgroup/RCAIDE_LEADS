# RCAIDE/Library/Components/Powertrain/Systems/Furnishings.py
# 
# Created:  Jan 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems import Systems

# ----------------------------------------------------------------------
# Furnishings
# ----------------------------------------------------------------------
class Furnishings(Systems):
    """
    A class representing a furnishings component used to model the mass and volume of furnishings.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Systems.Systems
    """
    def __defaults__(self):
        self.tag        = 'furnishings'