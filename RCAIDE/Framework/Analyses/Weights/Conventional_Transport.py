# RCAIDE/Framework/Analyses/Weights/Conventional_Transport.py
#
# Created:  Jun 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .Conventional import Conventional

# ----------------------------------------------------------------------------------------------------------------------
#  Transport Transport Weights Analysis
# ----------------------------------------------------------------------------------------------------------------------
class Conventional_Transport(Conventional):
    """ This is class that evaluates the weight of a conventional tube and wing transport class aircraft

    Assumptions:
        None

    Source:
        N/A

    Inputs:
        None

    Outputs:
        None

    Properties Used:
         N/A
    """

    def __defaults__(self):
        """This sets the default values and methods for the tube and wing
        aircraft weight analysis.

        Assumptions:
        None

        Source:
        N/A

        Inputs:
        None

        Outputs:
        None

        Properties Used:
        N/A
        """ 
        self.aircraft_type                                 = 'Transport' 