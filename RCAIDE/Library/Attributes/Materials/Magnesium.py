# Magnesium.py
#
# Created: Jul, 2022, J. Smart
# Modified:

# -------------------------------------------------------------------------------
# Imports
# -------------------------------------------------------------------------------

from .Solid import Solid
from RCAIDE.Framework.Core import Units


# -------------------------------------------------------------------------------
# Aluminum 6061-T6 Solid Class
# ------------------------------------------------------------------------------- 
class Magnesium(Solid):
    """
    A class representing RZ5 magnesium alloy material properties per BS 2L.128 standard.

    Attributes
    ----------
    ultimate_tensile_strength : float
        Maximum tensile stress before failure in Pa (200e6)
    ultimate_shear_strength : float
        Maximum shear stress before failure in Pa (138e6)
    ultimate_bearing_strength : float
        Maximum bearing stress before failure in Pa (330e6)
    yield_tensile_strength : float
        Stress at which material begins to deform plastically in Pa (135e6)
    yield_shear_strength : float
        Shear stress at which material begins to deform plastically in Pa (138e6)
    yield_bearing_strength : float
        Bearing stress at which material begins to deform plastically in Pa (130e6)
    minimum_gage_thickness : float
        Minimum manufacturable thickness in m (0.0)
    density : float
        Material density in kg/m³ (1840)

    Notes
    -----
    This class implements material properties for RZ5 magnesium alloy, which is commonly 
    used in aerospace applications due to its high strength-to-weight ratio. The zero 
    value for minimum gage thickness indicates that this should be specified based on 
    specific manufacturing capabilities.

    **Definitions**
    
    'RZ5'
        A magnesium alloy designation per British Standard BS 2L.128, containing zinc 
        and rare earth elements for improved strength
    
    'Ultimate Strength'
        The maximum stress that a material can withstand before failure
    
    'Yield Strength'
        The stress at which a material begins to deform plastically

    References
    ----------
    [1] MatWeb. (n.d.). Magnesium Elektron Elektron® RZ5 Magnesium Casting Alloy, UNS M16410. Magnesium elektron elektron® rz5 magnesium casting alloy, uns m16410. https://www.matweb.com/search/datasheet.aspx?matguid=f473a6bbcabe4fd49199c2cef7205664 
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

        self.ultimate_tensile_strength  = 200e6 * Units.Pa
        self.ultimate_shear_strength    = 138e6 * Units.Pa
        self.ultimate_bearing_strength  = 330e6 * Units.Pa
        self.yield_tensile_strength     = 135e6 * Units.Pa
        self.yield_shear_strength       = 138e6 * Units.Pa
        self.yield_bearing_strength     = 130e6 * Units.Pa
        self.minimum_gage_thickness     = 0.0   * Units.m
        self.density                    = 1840. * Units['kg/(m**3)']
