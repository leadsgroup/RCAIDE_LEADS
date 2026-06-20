# RCAIDE/Library/Components/Powertrain/Systems/Environmental_Controls.py
# 
# Created:  Jan 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems import Systems
from RCAIDE.Library.Methods.Powertrain.Systems.append_systems_conditions import append_systems_conditions
from RCAIDE.Library.Methods.Powertrain.Systems.compute_systems_power_draw import compute_systems_power_draw
 
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
    