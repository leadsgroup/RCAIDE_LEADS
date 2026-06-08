# RCAIDE/Library/Components/Powertrain/Systems/Hydraulics.py
# 
# Created:  Jan 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports
from RCAIDE.Framework.Core import Data
from .Systems import Systems
from RCAIDE.Library.Methods.Powertrain.Systems.append_systems_conditions import append_systems_conditions
 
# ----------------------------------------------------------------------------------------------------------------------
#  Hydraulics
# ----------------------------------------------------------------------------------------------------------------------            
class Hydraulics(Systems):
    """
    A class representing hydraulic systems and their power requirements. 
    """        
    def __defaults__(self):
        """
        Sets default values for the hydraulic system attributes.
        """                  
        self.tag                            = 'hydraulic'
        self.left_system                    = Data()
        self.left_system.number_of_pumps    = 1
        self.left_system.flowspeed          = 140.0
        self.left_system.system_power       = 204.0
        self.right_system                   = Data()
        self.right_system.number_of_pumps   = 1
        self.right_system.flowspeed         = 140.0
        self.right_system.system_power      = 204.0
        self.central_system                 = Data()
        self.central_system.number_of_pumps =  1
        self.central_system.flowspeed       =  23.0
        self.central_system.system_power    = 196.0
        
        
    def append_operating_conditions(self, segment, bus): 
        """
        Adds operating conditions for the hydraulic systems system to a mission segment.

        Parameters
        ----------
        segment : Data
            Mission segment to which conditions are being added
        bus : Data
            Electrical bus supplying power to the hydraulic systems
        """
        append_systems_conditions(self, segment, bus)
        
        return