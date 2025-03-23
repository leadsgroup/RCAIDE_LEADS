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
    """ Electric Network Class - Derivative of the hybrid energy network class
                               
    Attributes
    ----------
    tag : str
        Identifier for the network
        
    See Also
    --------
    RCAIDE.Library.Framework.Networks.Fuel
        Fuel network class 
    RCAIDE.Library.Framework.Networks.Hybrid
        Hybrid network class  
    """      
    def __defaults__(self):
        """ This sets the default values for the network to function.

            Assumptions:
            None

            Source:
            N/A 
        """         

        self.tag                          = 'electric' 