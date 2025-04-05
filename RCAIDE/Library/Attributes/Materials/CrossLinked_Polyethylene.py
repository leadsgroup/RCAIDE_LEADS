# RCAIDE/Library/Attributes/Solids/CrossLinked_Polyethylene.py
# 

# Created: Jan 2025 M. Clarke

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------

from .Solid import Solid 
from array import * 

#-------------------------------------------------------------------------------
# CrossLinked_Polyethylene Insulation Material
#------------------------------------------------------------------------------- 
class CrossLinked_Polyethylene(Solid): 
    """ Physical constants specific to CrossLinked_Polyethylene (XLPE)
    """

    def __defaults__(self):
        """Sets material properties at instantiation. 

        Assumptions:
            None
    
        Source:
            Guo, Ruochen, et al. "Electrical Architecture of 90-seater Electric Aircraft: A Cable Perspective."
            IEEE Transactions on Transportation Electrification (2024).
        """
        self.electrical_permittivity    = 2.3
        self.dielectric_strength_range  = [3.5E7,5E7]  
        self.density                    = 930
        self.thermal_conductivity       = 0.29
        self.melting_point              = 403  
        self.temperature_range          = [233, 363]  
        self.modulus_of_elasticity      = 0.6E9
        self.yield_tensile_strength     = 18E6 
        return 