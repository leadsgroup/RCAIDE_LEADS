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
from RCAIDE.Library.Methods.Powertrain.Systems.compute_hydraulics_power_draw import compute_hydraulics_power_draw
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
    
    def compute_performance(self,state,bus):   
        """
        Computes the power draw of the hydraulic systems based on the operating conditions."""

        compute_hydraulics_power_draw(self,state,bus)   

        return  