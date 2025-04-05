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
    """ Fuel Network Class - Derivative of the hybrid energy network class
                               
    Attributes
    ----------
    tag : str
        Identifier for the network 
        
    See Also
    --------
    RCAIDE.Library.Framework.Networks.Hybrid
        Hybrid network class 
    RCAIDE.Library.Framework.Networks.Electric
        All-Electric network class  
    """      
    def __defaults__(self):
        """ This sets the default values for the network to function. 
        """  
        self.tag                          = 'fuel' 