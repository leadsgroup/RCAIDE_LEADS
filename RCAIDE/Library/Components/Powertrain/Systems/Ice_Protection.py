# RCAIDE/Library/Components/Powertrain/Systems/Ice_Protection.py
# 
# Created:  May 2026, M. Clarke, S. Sharma

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems import Systems
from RCAIDE.Library.Methods.Powertrain.Systems.append_systems_conditions import append_systems_conditions
 
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
        
    def append_operating_conditions(self, segment, bus): 
        """
        Adds operating conditions for the ice protection system to a mission segment.

        Parameters
        ----------
        segment : Data
            Mission segment to which conditions are being added
        bus : Data
            Electrical bus supplying power to the avionics
        """
        append_systems_conditions(self, segment, bus)
        return         
    
        
