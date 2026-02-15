# RCAIDE/Library/Components/Powertrain/Systems/Instruments.py
# 
# Created:  Jan 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems import Systems
 
# ----------------------------------------------------------------------------------------------------------------------
#  Instruments
# ----------------------------------------------------------------------------------------------------------------------            
class Instruments(Systems):
    """
    A class representing instruments systems and their power requirements. 
    """        
    def __defaults__(self):
        """
        Sets default values for the instruments system attributes.
        """                  
        self.tag        = 'instruments' 