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
    """
    A class representing Perfluoroalkoxy(PFA) and its material properties.

    Attributes
    ----------
    electrical_permittivity : float
        Material electrical permittivity in kg/m³ (2.1)
    dielectric_strength_range : list
        Range of dielectric strength in Pa (7E7,8E7)
    density : float
        Material density in kg/m³ (2150)
    thermal_conductivity : float
        Heat conduction coefficient in W/(m·K) (0.19)
    melting_point : float
        Material melting point in K (578)
    temperature_range : list
        Range of temperature in K (183, 533)
    modulus_of_elasticity : float
        Material modulus of elasticity in Pa (0.55E9)
    yield_tensile_strength : float
        Stress at which material begins to deform plastically in Pa (28e6)
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