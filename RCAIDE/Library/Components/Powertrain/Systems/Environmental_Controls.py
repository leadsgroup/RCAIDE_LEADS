# RCAIDE/Library/Components/Powertrain/Systems/Environmental_Control_System.py
# 
# Created:  Oct 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems import Systems 
 
# ----------------------------------------------------------------------------------------------------------------------
# Environmental Control System
# ----------------------------------------------------------------------------------------------------------------------            
class Environmental_Controls(Systems):
    """

    """  
    def __defaults__(self): 
        """
        Sets default values for the system attributes.
        """        
        self.tag                   = 'environmental_control_system'  