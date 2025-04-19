# RCAIDE/Energy/Networks/Human_Powered.py
#  
# Created:  Sep 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
from .Network                           import Network 

# ----------------------------------------------------------------------------------------------------------------------
#  Human Powered
# ----------------------------------------------------------------------------------------------------------------------  
class Human_Powered(Network):
    """  A human powered system with batteries and maximum power point tracking.
    
        Assumptions:
        None
        
        Source:
        None
    """  
    def __defaults__(self):
        """ This sets the default values for the network to function.

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

        self.tag                          = 'solar'
        self.system_voltage               = None   
        self.reverse_thrust               = False
        self.wing_mounted                 = True        
