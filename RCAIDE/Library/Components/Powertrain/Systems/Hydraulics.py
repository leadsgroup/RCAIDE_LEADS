# RCAIDE/Library/Components/Powertrain/Systems/Hydraulics.py
# 
# Created:  Jan 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems import Systems
 
# ----------------------------------------------------------------------------------------------------------------------
#  Hydraulics
# ----------------------------------------------------------------------------------------------------------------------            
class Hydraulics(Systems):
    """
    A class representing hydraulic systems and their power requirements. 
    """        
    def __defaults__(self):
        """
        Sets default values for the hydraulic system attributes.
        """                  
        self.tag        = 'hydraulic' 