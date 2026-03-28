# RCAIDE/Framework/Energy/Networks/Fuel.py
# 
# Created:  Oct 2023, M. Clarke
#           Jan 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  Imports
# ---------------------------------------------------------------------------------------------------------------------
# RCAIDE Imports  
from .Network                                              import Network

# ----------------------------------------------------------------------------------------------------------------------
# Fuel
# ----------------------------------------------------------------------------------------------------------------------  
class Fuel(Network):
    """ Fuel Network Class - Derivative of the hybrid energy network class
                               
    Attributes
    ----------
    tag : str
        Identifier for the network 
        
    See Also
    --------
    RCAIDE.Library.Framework.Networks.Hybrid
        Hybrid network class 
    RCAIDE.Library.Framework.Networks.Fuel_Cell
        Fuel Cell network class
    RCAIDE.Library.Framework.Networks.Electric
        All-Electric network class  
    RCAIDE.Library.Framework.Networks.Hydrogen
        Hydrogen network class
    """
    def __defaults__(self):
        """ This sets the default values for the network to function. 
        """  
        self.tag                                 = 'fuel' 
        self.hybrid_power_split_ratio            = 1.0
        self.battery_fuel_cell_power_split_ratio = 1.0       