# Nickel.py
#
# Created: Jul, 2017, J. Smart
# Modified: Apr, 2018, J. Smart

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------

from .Solid import Solid
from RCAIDE.Framework.Core import Units

#-------------------------------------------------------------------------------
# Cold Rolled Nickel/Cobalt Chromoly Alloy Solid Class
#-------------------------------------------------------------------------------

class Nickel(Solid):
    """
    A class representing nickel alloy material properties.

    Attributes
    ----------
    ultimate_tensile_strength : float
        Maximum tensile stress before failure in Pa (1830e6)
    ultimate_shear_strength : float
        Maximum shear stress before failure in Pa (1050e6)
    ultimate_bearing_strength : float
        Maximum bearing stress before failure in Pa (1830e6)
    yield_tensile_strength : float
        Stress at which material begins to deform plastically in Pa (1550e6)
    yield_shear_strength : float
        Shear stress at which material begins to deform plastically in Pa (1050e6)
    yield_bearing_strength : float
        Bearing stress at which material begins to deform plastically in Pa (1550e6)
    minimum_gage_thickness : float
        Minimum manufacturable thickness in m (0.0)
    density : float
        Material density in kg/m³ (8430)

    Notes
    -----
    This class implements material properties for nickel alloy based on 
    median values from manufacturer reported data. The zero value for minimum gage thickness 
    indicates that this should be specified based on specific manufacturing capabilities.

    **Definitions**
    
    'Ultimate Strength'
        The maximum stress that a material can withstand before failure
    
    'Yield Strength'
        The stress at which a material begins to deform plastically

    References
    ----------
    [1] MatWeb. (n.d.).  Overview of materials for Nickel Alloy. Overview of materials for nickel alloy. https://www.matweb.com/search/DataSheet.aspx?MatGUID=8808b026f7c14d2f8d61f2d476aaeb26 
    """
    def __defaults__(self):
        """Sets material properties at instantiation.

        Assumptions:
        None

        Source:
        N/A

        Inputs:
        N/A

        Outputs:
        N/A

        Properties Used:
        None
        """

        self.ultimate_tensile_strength  = 1830e6    * Units.Pa
        self.ultimate_shear_strength    = 1050e6    * Units.Pa
        self.ultimate_bearing_strength  = 1830e6    * Units.Pa
        self.yield_tensile_strength     = 1550e6    * Units.Pa
        self.yield_shear_strength       = 1050e6    * Units.Pa
        self.yield_bearing_strength     = 1550e6    * Units.Pa
        self.minimum_gage_thickness     = 0.0       * Units.m
        self.density                    = 8430.     * Units['kg/(m**3)']
