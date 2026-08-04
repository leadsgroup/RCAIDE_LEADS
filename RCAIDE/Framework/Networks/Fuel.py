# RCAIDE/Framework/Networks/Fuel.py
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
    """ Fuel Network Class - Derivative of the base energy network class

    Attributes
    ----------
    tag : str
        Identifier for the network

    See Also
    --------
    RCAIDE.Framework.Networks.Electric
        All-Electric network class
    """
    def __defaults__(self):
        """ This sets the default values for the network to function. 
        """  
        self.tag                                 = 'fuel'