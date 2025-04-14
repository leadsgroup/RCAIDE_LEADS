# Carbon_Fiber_Honeycomb.py
#
# Created: Jul, 2017, J. Smart
# Modified: Apr, 2018, J. Smart

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------

from .Solid import Solid
from RCAIDE.Framework.Core import Units

#-------------------------------------------------------------------------------
# Carbon Fiber Honeycomb Core Solid Class
#-------------------------------------------------------------------------------

class Carbon_Fiber_Honeycomb(Solid):
    """
    A class representing carbon fiber honeycomb core material properties.

    Attributes
    ----------
    ultimate_tensile_strength : float
        Maximum tensile stress before failure in Pa (1e6)
    ultimate_shear_strength : float
        Maximum shear stress before failure in Pa (1e6)
    ultimate_bearing_strength : float
        Maximum bearing stress before failure in Pa (1e6)
    yield_tensile_strength : float
        Stress at which material begins to deform plastically in Pa (1e6)
    yield_shear_strength : float
        Shear stress at which material begins to deform plastically in Pa (1e6)
    yield_bearing_strength : float
        Bearing stress at which material begins to deform plastically in Pa (1e6)
    minimum_gage_thickness : float
        Minimum manufacturable thickness in m (6.5e-3)
    density : float
        Material density in kg/m³ (55.0)

    Notes
    -----
    This class implements material properties for carbon fiber honeycomb core materials
    typically used in sandwich composite structures. The low density and relatively high
    thickness reflect its primary use as a lightweight core material.

    **Definitions**
    
    'Ultimate Strength'
        The maximum stress that a material can withstand before failure
    
    'Yield Strength'
        The stress at which a material begins to deform plastically

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

        self.ultimate_tensile_strength  = 1e6       * Units.Pa
        self.ultimate_shear_strength    = 1e6       * Units.Pa
        self.ultimate_bearing_strength  = 1e6       * Units.Pa
        self.yield_tensile_strength     = 1e6       * Units.Pa
        self.yield_shear_strength       = 1e6       * Units.Pa
        self.yield_bearing_strength     = 1e6       * Units.Pa
        self.minimum_gage_thickness     = 6.5e-3    * Units.m
        self.density                    = 55.       * Units['kg/(m**3)']
