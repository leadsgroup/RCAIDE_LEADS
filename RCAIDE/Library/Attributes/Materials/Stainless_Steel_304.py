# RCAIDE/Library/Attributes/Solids/Stainless_Steel_304.py
# 
# Created:  
#
#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from .Solid import Solid 

#-------------------------------------------------------------------------------
# Stainless Steel 304
#------------------------------------------------------------------------------- 
class Stainless_Steel_304(Solid): 
    """ 
    A class representing stainless steel 304 material properties for aerospace 
    and cryogenic applications.  

    Attributes
    ----------
    density : float
        Material density in kg/m³ (8000).  
    thermal_conductivity : float
        Heat conduction coefficient in W/(m·K) (≈1 at cryogenic temperatures).
    yield_tensile_strength : float
        Stress at which material begins to deform plastically in Pa (548e6).  

    Notes
    -----
    Stainless steel 304 is an austenitic chromium-nickel alloy commonly used in 
    cryogenic storage tanks, aerospace structures, and thermal systems. It offers 
    excellent corrosion resistance and strength retention at low temperatures, 
    but has lower thermal conductivity compared to aluminum alloys.  

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
        Uses representative values for stainless steel 304 at cryogenic temperatures.  
        """
        self.density                 = 8.0e3 * Units['kg/(m**3)']        
        self.thermal_conductivity    = 1.0
        self.yield_tensile_strength  = 548e6 * Units.Pa