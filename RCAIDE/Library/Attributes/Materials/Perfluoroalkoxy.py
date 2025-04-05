# RCAIDE/Library/Attributes/Solids/Perfluoroalkoxy.py
# 

# Created: Jan 2025 M. Clarke

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------

from .Solid import Solid 
from array import * 

#-------------------------------------------------------------------------------
# Perfluoroalkoxy Insulation Material
#------------------------------------------------------------------------------- 
class Perfluoroalkoxy(Solid): 
    """ Physical constants specific to Perfluoroalkoxy(PFA) 
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
        self.dielectric_strength_range  = [7E7, 8E7]  # [V/m] 
        self.density                    = 2150
        self.thermal_conductivity       = 0.19
        self.melting_point              = 578
        self.temperature_range          = [183, 533]
        self.modulus_of_elasticity      = 0.55E9 
        self.yield_tensile_strength     = 28E6
        return 