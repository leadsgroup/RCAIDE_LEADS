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
        
    def append_operating_conditions(self, segment, bus): 
        """
        Adds operating conditions for the avionics system to a mission segment.

        Parameters
        ----------
        segment : Data
            Mission segment to which conditions are being added
        bus : Data
            Electrical bus supplying power to the avionics
        """
        append_systems_conditions(self, segment, bus)
        return         
    
        
