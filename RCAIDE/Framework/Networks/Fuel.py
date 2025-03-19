# RCAIDE/Framework/Energy/Networks/Fuel.py
# 
# Created:  Oct 2023, M. Clarke
#           Jan 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  Imports
# ---------------------------------------------------------------------------------------------------------------------
# RCAIDE Imports  
from .Hybrid                                              import Hybrid

# ----------------------------------------------------------------------------------------------------------------------
# Fuel
# ----------------------------------------------------------------------------------------------------------------------  
class Fuel(Hybrid):
    """ Fuel Network Class - Derivative of the Hybrid Energy Network Class
    """  
    def __defaults__(self):
        """ This sets the default values for the network to function. 
        """  
        self.tag                          = 'fuel'
        self.hybrid_power_split_ratio     = 0.0