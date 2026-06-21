# RCAIDE/Library/Components/Powertrain/Systems/Auxiliary_Power_Unit.py
# 
# Created:  Jan 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems import Systems
 
# ----------------------------------------------------------------------
# Auxiliary_Power_Unit
# ----------------------------------------------------------------------
class Auxiliary_Power_Unit(Systems): 
    """
    A class representing auxiliary power unit and their power requirements. 
    """        
    def __defaults__(self):
        """
        Sets default values for the auxiliary power unit attributes.
        """                  
        self.tag        = 'auxiliary_power_unit'
    