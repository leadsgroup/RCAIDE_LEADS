# RCAIDE/Library/Components/Powertrain/Systems/Auxiliary_Power_Unit.py
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
class Auxiliary_Power_Unit(Systems):
    """
    A class representing the apu. 
    """        
    def __defaults__(self):
        """
        Sets default values for the instruments system attributes.
        """                  
        self.tag        = 'apu' 