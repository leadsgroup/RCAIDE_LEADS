# RCAIDE/Framework/Analyses/Weights/Conventional_BWB.py
#
# Created:  Jun 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .Conventional import Conventional

# ----------------------------------------------------------------------------------------------------------------------
#  BWB Transport Weights Analysis
# ----------------------------------------------------------------------------------------------------------------------
class Conventional_BWB(Conventional):
    """ This is class that evaluates the weight of a conventional blended wing body class aircraft

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
        self.aircraft_type                                 = 'BWB' 