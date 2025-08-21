# RCAIDE/Library/Attributes/Solids/Aluminum_2219.py
# 
# Created: Aug 2025, S. Shekar
#
#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from .Solid import Solid 

#-------------------------------------------------------------------------------
# Aluminum 2219
#------------------------------------------------------------------------------- 
class Aluminum_2219(Solid): 
    """ 
    A class representing aluminum alloy 2219 properties, used in aerospace 
    structures for its strength and good thermal characteristics.  

    Attributes
    ----------
    density : float
        Material density in kg/m³ (3000).  
    thermal_conductivity : float
        Heat conduction coefficient in W/(m·K) (25).  
    yield_tensile_strength : float
        Stress at which material begins to plastically deform in Pa 
        (500e6).  
    
    Notes
    -----
    Aluminum 2219 is a high-strength, heat-treatable alloy with good 
    cryogenic properties, making it widely used in aerospace fuel tanks, 
    pressure vessels, and structural components.  

    **Definitions**

    'Yield Strength'
        The stress at which a material begins to plastically deform.  

    'Ultimate Strength'
        The maximum stress that a material can withstand before fracture.  

    'Thermal Conductivity'
        The property of a material to conduct heat, measured in watts per 
        meter-kelvin.  
    """

    def __defaults__(self):
        """
        Sets material properties at instantiation. 

        Parameters
        ----------
        None

        Returns
        -------
        None

        Notes
        -----
        * Uses typical values for Aluminum 2219 in aerospace applications.  
        * Strength values may vary depending on heat treatment condition 
          (e.g., T6, T851).  
        """
        self.density                    = 3.0e3 * Units['kg/(m**3)']        
        self.thermal_conductivity       = 25
        self.yield_tensile_strength     = 500e6 * Units.Pa
