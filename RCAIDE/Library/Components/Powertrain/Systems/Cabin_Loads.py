# RCAIDE/Library/Components/Powertrain/Systems/Cabin_Loads.py
# 
# Created:  May 2026, M. Clarke, S. Sharma

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems import Systems
from RCAIDE.Library.Methods.Powertrain.Systems.append_systems_conditions import append_systems_conditions
from RCAIDE.Library.Methods.Powertrain.Systems.compute_cabin_loads_power_draw import compute_cabin_loads_power_draw

# ----------------------------------------------------------------------------------------------------------------------
#  Cabin_Loads
# ----------------------------------------------------------------------------------------------------------------------
class Cabin_Loads(Systems):
    """
    A class representing cabin loads and their power requirements.
    """
    def __defaults__(self):
        self.tag        = 'cabin_loads' 

    def compute_performance(self, vehicle,state,bus):
        compute_cabin_loads_power_draw(self, vehicle,state,bus)
        return