# RCAIDE/Library/Attributes/Materials/Aluminum_2219_Alloy.py
#
# Created: Aug 2025, S. Shekar
# Updated: May 2026, M. Clarke

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from .Solid import Solid

#-------------------------------------------------------------------------------
# Aluminum 2219-T87
#-------------------------------------------------------------------------------
class Aluminum_2219_Alloy(Solid):
    """
    A class representing aluminum alloy 2219-T87 and its material properties.
    Used in aerospace structures and cryogenic fuel tanks for its strength and
    good weldability at elevated and cryogenic temperatures.

    Attributes
    ----------
    density : float
        Material density in kg/m³ (2840)
    thermal_conductivity : float
        Heat conduction coefficient in W/(m·K) (121)
    specific_heat_capacity : float
        Specific heat at constant pressure in J/(kg·K) (864)
    ultimate_tensile_strength : float
        Maximum tensile stress before failure in Pa (476e6)
    ultimate_shear_strength : float
        Maximum shear stress before failure in Pa (283e6)
    ultimate_bearing_strength : float
        Maximum bearing stress before failure in Pa (689e6)
    yield_tensile_strength : float
        Stress at which material begins to deform plastically in Pa (393e6)
    yield_shear_strength : float
        Shear stress at which material begins to deform plastically in Pa (228e6)
    yield_bearing_strength : float
        Bearing stress at which material begins to deform plastically in Pa (517e6)
    youngs_modulus : float
        Modulus of elasticity in Pa (73.8e9)
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
    Aluminum 2219 is a high-strength, heat-treatable alloy with good cryogenic
    properties, making it widely used in aerospace fuel tanks (Space Shuttle
    external tank, Saturn V), pressure vessels, and structural components.
    Properties correspond to the T87 temper (solution heat-treated, cold-worked,
    then artificially aged).

    **Definitions**

    'Ultimate Strength'
        The maximum stress that a material can withstand before failure

    'Yield Strength'
        The stress at which a material begins to deform plastically

    'Thermal Conductivity'
        The property of a material to conduct heat, measured in watts per meter-kelvin

    References
    ----------
    [1] MatWeb. (n.d.). Aluminum 2219-T87.
        https://www.matweb.com/search/DataSheet.aspx?MatGUID=4af375b2de9d44ba85a0e8c1f21d7b2e
    [2] MMPDS-01. (2003). Metallic Materials Properties Development and
        Standardization. FAA.
    """

    def __defaults__(self):
        """Sets material properties at instantiation.

        Assumptions:
            T87 temper (solution heat-treated, cold-worked, artificially aged).

        Source:
            MatWeb Aluminum 2219-T87; MMPDS-01.
        """

        self.density                    = 2840.  * Units['kg/(m**3)']
        self.thermal_conductivity       = 121.0
        self.specific_heat_capacity     = 864
        self.ultimate_tensile_strength  = 476e6  * Units.Pa
        self.ultimate_shear_strength    = 283e6  * Units.Pa
        self.ultimate_bearing_strength  = 689e6  * Units.Pa
        self.yield_tensile_strength     = 393e6  * Units.Pa
        self.yield_shear_strength       = 228e6  * Units.Pa
        self.yield_bearing_strength     = 517e6  * Units.Pa
        self.youngs_modulus             = 73.8e9 * Units.Pa
        self.poissons_ratio             = 0.33
        self.shear_modulus              = self.youngs_modulus / (2 * (1 + self.poissons_ratio))
        self.minimum_gage_thickness     = 1.5e-3 * Units.m
        self.minimum_width              = 25.4e-3 * Units.m
