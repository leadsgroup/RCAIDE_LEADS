# RCAIDE/Library/Components/Powertrain/Systems/Environmental_Controls.py
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
class Auxillary_Power_Unit(Systems): 
    """
    A class representing auxillary power unit and their power requirements. 
    """        
    def __defaults__(self):
        """
        Sets default values for the auxillary power unit attributes.
        """                  
        self.tag        = 'auxillary_power_unit'
