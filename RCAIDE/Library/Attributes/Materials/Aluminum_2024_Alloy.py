# RCAIDE/Library/Attributes/Materials/Aluminum_2024_Alloy.py
#
# Created: May 2026, M. Clarke

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from .Solid import Solid

#-------------------------------------------------------------------------------
# Aluminum 2024-T3
#-------------------------------------------------------------------------------
class Aluminum_2024_Alloy(Solid):
    """
    A class representing aluminum alloy 2024-T3 and its material properties.
    A copper-based high-strength alloy with excellent fatigue resistance,
    historically the most widely used material for aircraft wing and fuselage
    skins on commercial transport aircraft.

    Attributes
    ----------
    density : float
        Material density in kg/m³ (2780)
    thermal_conductivity : float
        Heat conduction coefficient in W/(m·K) (121)
    specific_heat_capacity : float
        Specific heat at constant pressure in J/(kg·K) (875)
    ultimate_tensile_strength : float
        Maximum tensile stress before failure in Pa (483e6)
    ultimate_shear_strength : float
        Maximum shear stress before failure in Pa (283e6)
    ultimate_bearing_strength : float
        Maximum bearing stress before failure in Pa (786e6)
    yield_tensile_strength : float
        Stress at which material begins to deform plastically in Pa (345e6)
    yield_shear_strength : float
        Shear stress at which material begins to deform plastically in Pa (193e6)
    yield_bearing_strength : float
        Bearing stress at which material begins to deform plastically in Pa (524e6)
    youngs_modulus : float
        Modulus of elasticity in Pa (73.1e9)
    poissons_ratio : float
        Ratio of transverse to axial strain (0.33)
    shear_modulus : float
        Shear modulus derived from E and nu in Pa
    minimum_gage_thickness : float
        Minimum manufacturable thickness in m (1.0e-3)
    minimum_width : float
        Minimum width in m (25.4e-3)

    Notes
    -----
    2024-T3 dominated lower wing skins and fuselage panels on aircraft such as
    the Boeing 707, 727, 737 (Classic/NG), and 747 owing to its superior fatigue
    crack-growth resistance compared to 7075. It is not weldable; riveted
    construction is standard. Clad sheet (Alclad 2024) is often used to improve
    corrosion resistance.

    **Definitions**

    'Ultimate Strength'
        The maximum stress that a material can withstand before failure

    'Yield Strength'
        The stress at which a material begins to deform plastically

    'Thermal Conductivity'
        The property of a material to conduct heat, measured in watts per meter-kelvin

    References
    ----------
    [1] MatWeb. (n.d.). Aluminum 2024-T3.
        https://www.matweb.com/search/DataSheet.aspx?MatGUID=8ac3ebccfb5b4c0f9a1b9cf21c9285df
    [2] MMPDS-01. (2003). Metallic Materials Properties Development and
        Standardization. FAA.
    """

    def __defaults__(self):
        """Sets material properties at instantiation.

        Assumptions:
            T3 temper (solution heat-treated, cold-worked, naturally aged).

        Source:
            MatWeb Aluminum 2024-T3; MMPDS-01.
        """

        self.density                    = 2780.  * Units['kg/(m**3)']
        self.thermal_conductivity       = 121.0
        self.specific_heat_capacity     = 875
        self.ultimate_tensile_strength  = 483e6  * Units.Pa
        self.ultimate_shear_strength    = 283e6  * Units.Pa
        self.ultimate_bearing_strength  = 786e6  * Units.Pa
        self.yield_tensile_strength     = 345e6  * Units.Pa
        self.yield_shear_strength       = 193e6  * Units.Pa
        self.yield_bearing_strength     = 524e6  * Units.Pa
        self.youngs_modulus             = 73.1e9 * Units.Pa
        self.poissons_ratio             = 0.33
        self.shear_modulus              = self.youngs_modulus / (2 * (1 + self.poissons_ratio))
        self.minimum_gage_thickness     = 1.0e-3 * Units.m
        self.minimum_width              = 25.4e-3 * Units.m
