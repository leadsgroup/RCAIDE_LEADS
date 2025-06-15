# RCAIDE/Framework/Analyses/Weights/Electric_VTOL.py
#
# Created:  Feb 2025, S. Shekar
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

from .Electric import Electric

# ----------------------------------------------------------------------------------------------------------------------
# Electric VTOL Weights class 
# ----------------------------------------------------------------------------------------------------------------------
class Electric_VTOL(Electric):
    """ This is class that evaluates the weight of an electric VTOL class aircraft

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
        self.propulsion_architecture                       = 'Electric' 
        self.aircraft_type                                 = 'VTOL'
        

        
     