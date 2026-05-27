# RCAIDE/Library/Attributes/Materials/Aluminum_7075_Alloy.py
#
# Created: May 2026, M. Clarke

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from .Solid import Solid

#-------------------------------------------------------------------------------
# Aluminum 7075-T6
#-------------------------------------------------------------------------------
class Aluminum_7075_Alloy(Solid):
    """
    A class representing aluminum alloy 7075-T6 and its material properties.
    One of the highest-strength aluminum alloys available, widely used in
    aircraft structural components including wing spars, ribs, and skins on
    legacy commercial and military aircraft.

    Attributes
    ----------
    density : float
        Material density in kg/m³ (2810)
    thermal_conductivity : float
        Heat conduction coefficient in W/(m·K) (130)
    specific_heat_capacity : float
        Specific heat at constant pressure in J/(kg·K) (960)
    ultimate_tensile_strength : float
        Maximum tensile stress before failure in Pa (572e6)
    ultimate_shear_strength : float
        Maximum shear stress before failure in Pa (331e6)
    ultimate_bearing_strength : float
        Maximum bearing stress before failure in Pa (1020e6)
    yield_tensile_strength : float
        Stress at which material begins to deform plastically in Pa (503e6)
    yield_shear_strength : float
        Shear stress at which material begins to deform plastically in Pa (290e6)
    yield_bearing_strength : float
        Bearing stress at which material begins to deform plastically in Pa (807e6)
    youngs_modulus : float
        Modulus of elasticity in Pa (71.7e9)
    poissons_ratio : float
        Ratio of transverse to axial strain (0.33)
    shear_modulus : float
        Shear modulus derived from E and nu in Pa
    minimum_gage_thickness : float
        Minimum manufacturable thickness in m (1.5e-3)
    minimum_width : float
        Minimum width in m (25.4e-3)

    Notes
    -----
    7075-T6 is a zinc-based alloy with one of the highest strength-to-weight
    ratios among aluminum alloys. It was the dominant material for wing
    structures on aircraft such as the Boeing 737, 747 (early generations),
    and numerous military aircraft before the transition to CFRP composites.
    It has lower corrosion resistance than 6061 and is not weldable; fastened
    and bonded joints are standard.

    **Definitions**

    'Ultimate Strength'
        The maximum stress that a material can withstand before failure

    'Yield Strength'
        The stress at which a material begins to deform plastically

    'Thermal Conductivity'
        The property of a material to conduct heat, measured in watts per meter-kelvin

    References
    ----------
    [1] MatWeb. (n.d.). Aluminum 7075-T6; 7075-T651.
        https://www.matweb.com/search/DataSheet.aspx?MatGUID=4f19a42be94546b686bbf43f79c51b7d
    [2] MMPDS-01. (2003). Metallic Materials Properties Development and
        Standardization. FAA.
    """

    def __defaults__(self):
        """Sets material properties at instantiation.

        Assumptions:
            T6 temper (solution heat-treated and artificially aged).

        Source:
            MatWeb Aluminum 7075-T6; MMPDS-01.
        """

        self.density                    = 2810.  * Units['kg/(m**3)']
        self.thermal_conductivity       = 130.0
        self.specific_heat_capacity     = 960
        self.ultimate_tensile_strength  = 572e6  * Units.Pa
        self.ultimate_shear_strength    = 331e6  * Units.Pa
        self.ultimate_bearing_strength  = 1020e6 * Units.Pa
        self.yield_tensile_strength     = 503e6  * Units.Pa
        self.yield_shear_strength       = 290e6  * Units.Pa
        self.yield_bearing_strength     = 807e6  * Units.Pa
        self.youngs_modulus             = 71.7e9 * Units.Pa
        self.poissons_ratio             = 0.33
        self.shear_modulus              = self.youngs_modulus / (2 * (1 + self.poissons_ratio))
        self.minimum_gage_thickness     = 1.5e-3 * Units.m
        self.minimum_width              = 25.4e-3 * Units.m
