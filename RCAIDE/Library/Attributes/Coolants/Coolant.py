# RCAIDE/Library/Attributes/Coolants/Coolant.py
# 
#
# Created:  Mar 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 

from RCAIDE.Framework.Core import Data

# ---------------------------------------------------------------------------------------------------------------------- 
#  Class
# ----------------------------------------------------------------------------------------------------------------------  
class Coolant(Data):
    """Default class of a liquid coolant.
    """

    def __defaults__(self):
        """This sets the default values.

        Assumptions:
            None
    
        Source:
            None
        """
        self.tag                       = 'Coolant'
        self.density                   = 0.0                       # kg/m^3
        self.specific_heat_capacity    = 0.0                       # J/kg.K
        self.thermal_conductivity      = 0.0                       # W/m.K
        self.dynamic_viscosity         = 0.0                       # Pa.s
        self.temperatures              = Data()
