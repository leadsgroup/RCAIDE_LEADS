# RCAIDE/Library/Components/Powertrain/Systems/Ice_Protection.py
# 
# Created:  May 2026, M. Clarke, S. Sharma

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems import Systems
from RCAIDE.Library.Methods.Powertrain.Systems.append_systems_conditions import append_systems_conditions
from RCAIDE.Library.Methods.Powertrain.Systems.compute_ice_protection_power_draw import compute_ice_protection_power_draw
 
# ----------------------------------------------------------------------------------------------------------------------
#  Ice_Protection
# ----------------------------------------------------------------------------------------------------------------------            
class Ice_Protection(Systems):
    """
    A class representing ice protection systems and their power requirements. 
    """        
    def __defaults__(self):
        """
        Sets default values for the ice protection system attributes.
        """                  
        self.tag        = 'ice_protection' 

    def compute_performance(self, state, vehicle):
        inputs, outputs = compute_ice_protection_power_draw(self, state, vehicle)
        return inputs, outputs, False, None
