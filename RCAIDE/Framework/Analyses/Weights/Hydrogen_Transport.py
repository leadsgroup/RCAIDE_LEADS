# RCAIDE/Framework/Analyses/Weights/Hydrogen_Transport.py
#
# Created:  Jun 2025, M. Clarke
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .Hydrogen import Hydrogen

# ----------------------------------------------------------------------------------------------------------------------
#  Hydrogen Transport Class 
# ----------------------------------------------------------------------------------------------------------------------
class Hydrogen_Transport(Hydrogen):
    """ This is class that evaluates the weight of a hydrogen transport class aircraft

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