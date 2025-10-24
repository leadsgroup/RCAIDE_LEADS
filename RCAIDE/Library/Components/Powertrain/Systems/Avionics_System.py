# RCAIDE/Library/Components/Powertrain/Systems/Avionics_System.py
# 
# Created:  Oct 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .System     import System
from RCAIDE.Library.Methods.Powertrain.Systems.compute_systems_power_draw import compute_systems_power_draw
from RCAIDE.Library.Methods.Powertrain.Systems.append_systems_conditions import append_systems_conditions
 
# ----------------------------------------------------------------------------------------------------------------------
# Avionics System
# ----------------------------------------------------------------------------------------------------------------------            
class Avionics_System(System):
    """

    """  
    def __defaults__(self): 
        """
        Sets default values for the system attributes.
        """        
        self.tag                   = 'avionics_system' 

    def append_operating_conditions(self, segment): 
        """

        """
        append_systems_conditions(self, segment)
        return
    
    def compute_performance(self, state):

        inputs, outputs = compute_systems_power_draw(self, state)

        return inputs, outputs