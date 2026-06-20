# RCAIDE/Library/Components/Powertrain/Systems/Environmental_Controls.py
# 
# Created:  Jan 2026, M. Clarke 
# Modified: May 2026, S. Sharma

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems import Systems
from RCAIDE.Library.Methods.Powertrain.Systems.append_systems_conditions import append_systems_conditions
from RCAIDE.Library.Methods.Powertrain.Systems.compute_ecs_power_draw import compute_ecs_power_draw
 
# ----------------------------------------------------------------------------------------------------------------------
#  Environmental_Controls
# ----------------------------------------------------------------------------------------------------------------------            
class Environmental_Controls(Systems):
    """
    A class representing environmental control systems and their power requirements. 
    """        
    def __defaults__(self):
        """
        Sets default values for the environmental control system attributes.
        """                  
        self.tag        = 'environmental_controls' 
        self.cabin_compressor_efficiency   = 0.85  

    def compute_performance(self,vehicle, segment, bus):
        compute_ecs_power_draw(self, vehicle, segment, bus)
        return
