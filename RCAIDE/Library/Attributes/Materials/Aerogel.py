# RCAIDE/Library/Attributes/Solids/Aerogel.py
# 
# Created: Aug 2025, S. Shekar
#
#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from .Solid import Solid 

#-------------------------------------------------------------------------------
# Aerogel Material
#------------------------------------------------------------------------------- 
class Aerogel(Solid): 
    """ 
    A class representing aerogel material properties for aerospace thermal 
    protection and insulation applications.

    Attributes
    ----------
    density : float
        Material density in kg/m³ (130).
    thermal_conductivity : float
        Heat conduction coefficient in W/(m·K) (0.0113).
  
    Notes
    -----
    Aerogels are ultra-lightweight materials with extremely low density and 
    thermal conductivity, making them suitable for thermal insulation in 
    aerospace applications such as cryogenic fuel tanks, battery protection, 
    and thermal protection systems. However, their mechanical strength is 
    limited, so structural usage is not recommended.  

    **Definitions**

    'Aerogel'
        A class of synthetic porous ultralight materials derived from a gel, 
        where the liquid component is replaced with a gas.
    
    'Thermal Conductivity'
        The property of a material to conduct heat, measured in watts per 
        meter-kelvin.

    """

    def __defaults__(self):
        """
        Set default material properties at instantiation. 

        Parameters
        ----------
        None

        Returns
        -------
        None

        Notes
        -----
        * Assumes silica aerogel with typical bulk density and conductivity.
        * Mechanical strength values are not set due to brittleness of aerogels.
        """
        self.density                    = 130 * Units['kg/(m**3)']        
        self.thermal_conductivity       = 0.0113
