# RCAIDE/Energy/Networks/Electric.py
# 
# Created:  Jul 2023, M. Clarke
# Modified: Sep 2024, S. Shekar
#           Jan 2025, M. Clarke
  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE imports  
from .Hybrid import Hybrid 

# ----------------------------------------------------------------------------------------------------------------------
#  Electric
# ----------------------------------------------------------------------------------------------------------------------  
class Electric(Hybrid):
    """ Electric Network Class - Derivative of the Hybrid Energy Network Class
    """  
    def __defaults__(self):
        """ This sets the default values for the network to function. 
        """         
        self.tag                          = 'electric' 
        self.hybrid_power_split_ratio     = 0.0        