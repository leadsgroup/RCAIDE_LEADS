# RCAIDE/Energy/Networks/Fuel_Cell.py
# 
# Created: Apr 2025, M. Clarke
  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE imports 
from .Network import Network 

# ----------------------------------------------------------------------------------------------------------------------
#  Fuel_Cell
# ----------------------------------------------------------------------------------------------------------------------  
class Fuel_Cell(Network):
    """ Fuel Cell Network Class - Derivative of the hybrid energy network class
                               
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

        self.tag                          = 'fuel_cell' 