# RCAIDE/Library/Components/Powertrain/Systems/Pneumatic_System.py
# 
# Created:  Oct 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems     import Systems 
 
# ----------------------------------------------------------------------------------------------------------------------
# Pneumatic System
# ----------------------------------------------------------------------------------------------------------------------            
class Pneumatic(Systems):
    """

    """  
    def __defaults__(self): 
        """
        Sets default values for the system attributes.
        """        
        self.tag                   = 'pneumatic_system'  