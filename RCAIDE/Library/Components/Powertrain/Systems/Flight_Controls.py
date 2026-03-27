# RCAIDE/Library/Components/Powertrain/Systems/Flight_Controls.py
# 
# Created:  Jan 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems import Systems
 
# ----------------------------------------------------------------------------------------------------------------------
#  Flight_Controls
# ----------------------------------------------------------------------------------------------------------------------            
class Flight_Controls(Systems):
    """
    A class representing flight control systems and their power requirements. 
    """        
    def __defaults__(self):
        """
        Sets default values for the flight controls system attributes.
        """                  
        self.tag        = 'flight_controls' 