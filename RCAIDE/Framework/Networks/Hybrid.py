# RCAIDE/Energy/Networks/Hybrid.py
# 
# Created:  Jan 2025, M. Clarke
  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE imports 
from .Network import Network 

# ----------------------------------------------------------------------------------------------------------------------
#  Hybrid
# ----------------------------------------------------------------------------------------------------------------------  
class Hybrid(Network):
    """ Electric Network Class - Derivative of the hybrid energy network class
                               
    Attributes
    ----------
    tag : str
        Identifier for the network
        
    See Also
    --------
    RCAIDE.Library.Framework.Networks.Fuel
        Fuel network class 
    RCAIDE.Library.Framework.Networks.Electric
        Electric network class  
    RCAIDE.Library.Framework.Networks.Hydrogen
        Hydrogen network class
    RCAIDE.Library.Framework.Networks.Fuel_Cell
        Fuel Cell network class
    """
    def __defaults__(self):
        """ This sets the default values for the network to function.

            Assumptions:
            None

            Source:
            N/A 
        """         

        self.tag   = 'hybrid' 