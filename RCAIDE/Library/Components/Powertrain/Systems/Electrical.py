# RCAIDE/Library/Components/Powertrain/Systems/Electrical.py
# 
# Created:  Oct 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems     import Systems 
 
# ----------------------------------------------------------------------------------------------------------------------
# Electrical System
# ----------------------------------------------------------------------------------------------------------------------            
class Electrical(Systems):
    """

    """  
    def __defaults__(self): 
        """
        Sets default values for the system attributes.
        """        
        self.tag                   = 'electrical_system'  