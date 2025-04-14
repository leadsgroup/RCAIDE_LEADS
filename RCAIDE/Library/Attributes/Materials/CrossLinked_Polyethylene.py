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
    """
    A class representing CrossLinked_Polyethylene (XLPE) and its material properties.

    Attributes
    ----------
    electrical_permittivity : float
        Material electrical permittivity in kg/m³ (2.3)
    dielectric_strength_range : list
        Range of dielectric strength in Pa (3.5E7,5E7)
    density : float
        Material density in kg/m³ (930)
    thermal_conductivity : float
        Heat conduction coefficient in W/(m·K) (0.29)
    melting_point : float
        Material melting point in K (403)
    temperature_range : list
        Range of temperature in K (233, 363)
    modulus_of_elasticity : float
        Material modulus of elasticity in Pa (0.6E9)
    yield_tensile_strength : float
        Stress at which material begins to deform plastically in Pa (18e6)
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