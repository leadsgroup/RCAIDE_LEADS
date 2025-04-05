# RCAIDE/Library/Attributes/Solids/Polytetrafluoroethylene.py
# 

# Created: Jan 2025 M. Clarke

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------

from .Solid import Solid 
from array import * 

#-------------------------------------------------------------------------------
# Polytetrafluoroethylene Insulation Material
#------------------------------------------------------------------------------- 
class Polytetrafluoroethylene(Solid): 
    """ Physical constants specific to  Polytetrafluoroethylene (PFTE)
    """

    def __defaults__(self):
        """Sets material properties at instantiation. 

        Assumptions:
            None
    
        Source:
            Guo, Ruochen, et al. "Electrical Architecture of 90-seater Electric Aircraft: A Cable Perspective."
            IEEE Transactions on Transportation Electrification (2024).
        """
        self.electrical_permittivity    = 2.1
        self.dielectric_strength_range  = [6E7, 8E7] # [V/m]
        self.density                    = 2170
        self.thermal_conductivity       = 0.25
        self.melting_point              = 600 
        self.temperature_range          = [183 , 533]  
        self.modulus_of_elasticity      = 0.49E9
        self.yield_tensile_strength     = 24E6
        return 