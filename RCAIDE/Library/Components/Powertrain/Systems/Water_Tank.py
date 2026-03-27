# RCAIDE/Library/Components/Powertrain/Systems/Water_Tank.py
# 
# Created:  Jan 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems import Systems
 
# ----------------------------------------------------------------------
# Auxillary_Power_Unit
# ----------------------------------------------------------------------
class Water_Tank(Systems): 
    """
    A class representing auxillary power unit and their power requirements. 
    """        
    def __defaults__(self):
        """
        Sets default values for the auxillary power unit attributes.
        """                  
        self.tag        = 'water_tank'