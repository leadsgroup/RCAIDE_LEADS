# RCAIDE/Library/Components/Powertrain/Systems/Environmental_Controls.py
# 
# Created:  Jan 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems import Systems
from RCAIDE.Library.Methods.Powertrain.Systems.append_systems_conditions import append_systems_conditions
 
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
        
    def append_operating_conditions(self, segment, bus): 
        """
        Adds operating conditions for the auxillary power unit to a mission segment.

        Parameters
        ----------
        segment : Data
            Mission segment to which conditions are being added
        bus : Data
            Electrical bus supplying power to the auxillary power unit
        """
        append_systems_conditions(self, segment, bus)
        
        return         
    