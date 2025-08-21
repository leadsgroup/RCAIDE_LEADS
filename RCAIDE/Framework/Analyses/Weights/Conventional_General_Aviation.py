# RCAIDE/Framework/Analyses/Weights/Conventional_General_Aviationpy
#
# Created:  Jun 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .Conventional import Conventional

# ----------------------------------------------------------------------------------------------------------------------
#  General Aviation Transport Weights Analysis
# ----------------------------------------------------------------------------------------------------------------------
class Conventional_General_Aviation(Conventional):
    """ This is class that evaluates the weight of a conventional general aviation class aircraft

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
        self.aircraft_type                                 = 'General_Aviation' 