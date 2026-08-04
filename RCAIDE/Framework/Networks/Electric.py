# RCAIDE/Framework/Networks/Electric.py
#
# Created:  Oct 2023, M. Clarke
#           Jan 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  Imports
# ---------------------------------------------------------------------------------------------------------------------
# RCAIDE Imports
from .Network                                              import Network

# ----------------------------------------------------------------------------------------------------------------------
# Electric
# ----------------------------------------------------------------------------------------------------------------------
class Electric(Network):
    """ All-Electric Network Class - Derivative of the base energy network class

    Attributes
    ----------
    tag : str
        Identifier for the network

    See Also
    --------
    RCAIDE.Framework.Networks.Fuel
        Conventional/hybrid fuel network class
    """
    def __defaults__(self):
        """ This sets the default values for the network to function.

            Assumptions:
            None

            Source:
            N/A
        """

        self.tag                          = 'electric'