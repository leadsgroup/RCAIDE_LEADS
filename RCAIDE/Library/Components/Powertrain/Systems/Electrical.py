# RCAIDE/Library/Components/Powertrain/Systems/Electrical.py
# 
# Created:  Jan 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems import Systems
from RCAIDE.Library.Methods.Powertrain.Systems.append_systems_conditions import append_systems_conditions
from RCAIDE.Library.Methods.Powertrain.Systems.compute_systems_power_draw import compute_systems_power_draw
 
# ----------------------------------------------------------------------------------------------------------------------
#  Electrical
# ----------------------------------------------------------------------------------------------------------------------            
class Electrical(Systems):
    """
    A class representing electrical control systems and their power requirements. 
    """        
    def __defaults__(self):
        """
        Sets default values for the electrical system attributes.
        """                  
        self.tag        = 'electrical'  