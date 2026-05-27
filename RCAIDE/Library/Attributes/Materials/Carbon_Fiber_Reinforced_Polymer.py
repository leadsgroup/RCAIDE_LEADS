# RCAIDE/Library/Attributes/Materials/Carbon_Fiber_Reinforced_Polymer.py
#
# Created: May 2026, M. Clarke

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from .Solid import Solid

#-------------------------------------------------------------------------------
# Carbon Fiber Reinforced Polymer (CFRP)
#-------------------------------------------------------------------------------
class Carbon_Fiber_Reinforced_Polymer(Solid):
    """
    A class representing aerospace-grade carbon fiber reinforced polymer (CFRP)
    in a quasi-isotropic laminate configuration, representative of wing skin and
    spar structures used in modern aircraft such as the Boeing 787 and F-35.

    Attributes
    ----------
    density : float
        Material density in kg/m³ (1600)
    thermal_conductivity : float
        Heat conduction coefficient in W/(m·K) (7.0)
    specific_heat_capacity : float
        Specific heat at constant pressure in J/(kg·K) (900)
    ultimate_tensile_strength : float
        Maximum tensile stress before failure in Pa (600e6)
    ultimate_shear_strength : float
        Maximum in-plane shear stress before failure in Pa (90e6)
    ultimate_bearing_strength : float
        Maximum bearing stress before failure in Pa (700e6)
    yield_tensile_strength : float
        First-ply-failure tensile stress in Pa (570e6)
    yield_shear_strength : float
        First-ply-failure shear stress in Pa (75e6)
    yield_bearing_strength : float
        First-ply-failure bearing stress in Pa (490e6)
    youngs_modulus : float
        In-plane modulus of elasticity for quasi-isotropic laminate in Pa (70e9)
    poissons_ratio : float
        In-plane Poisson's ratio for quasi-isotropic laminate (0.30)
    shear_modulus : float
        In-plane shear modulus derived from E and nu in Pa
    minimum_gage_thickness : float
        Minimum manufacturable laminate thickness in m (0.5e-3)
    minimum_width : float
        Minimum width in m (25.4e-3)

    Notes
    -----
    Properties correspond to an IM7/8552 (or equivalent T800-series) prepreg
    cured quasi-isotropic laminate [0/±45/90]_ns. This stacking sequence is
    typical for primary wing structures and produces laminate-level properties
    that can be treated as effectively in-plane isotropic.

    CFRP does not exhibit a classical metal yield point; 'yield' values here
    represent first-ply-failure (FPF) onset, approximately 90–95 % of ultimate
    for a well-designed quasi-isotropic laminate.

    The shear modulus is computed from the quasi-isotropic approximation
    G = E / (2(1 + ν)), which is valid for this laminate type.

    **Definitions**

    'Ultimate Strength'
        The laminate-level stress at which catastrophic failure occurs

    'First-Ply Failure (FPF)'
        The stress at which the first individual ply reaches its failure criterion;
        used here as the composite analogue of yield strength

    'Quasi-Isotropic Laminate'
        A laminate whose in-plane stiffness and strength are approximately equal
        in all directions due to balanced, symmetric ply orientations

    References
    ----------
    [1] Hexcel Corporation. (2020). HexPly 8552 Product Data Sheet.
        https://www.hexcel.com/user_upload/assets/datasheets/Prepreg/8552.pdf
    [2] Tomblin, J., & Harter, P. (1999). AGATE composite material data.
        FAA/NASA National Center for Advanced Materials Performance (NCAMP).
    [3] Kassapoglou, C. (2013). Design and Analysis of Composite Structures.
        Wiley, 2nd ed.
    """

    def __defaults__(self):
        """Sets material properties at instantiation.

        Assumptions:
            Quasi-isotropic [0/±45/90]_ns laminate.
            First-ply-failure used as composite analogue of yield strength.

        Source:
            Hexcel HexPly 8552 data sheet; NCAMP IM7/8552 allowables;
            Kassapoglou (2013) Design and Analysis of Composite Structures.
        """

        self.density                    = 1600.  * Units['kg/(m**3)']
        self.thermal_conductivity       = 7.0
        self.specific_heat_capacity     = 900
        self.ultimate_tensile_strength  = 600e6  * Units.Pa
        self.ultimate_shear_strength    =  90e6  * Units.Pa
        self.ultimate_bearing_strength  = 700e6  * Units.Pa
        self.yield_tensile_strength     = 570e6  * Units.Pa
        self.yield_shear_strength       =  75e6  * Units.Pa
        self.yield_bearing_strength     = 490e6  * Units.Pa
        self.youngs_modulus             =  70e9  * Units.Pa
        self.poissons_ratio             = 0.30
        self.shear_modulus              = self.youngs_modulus / (2 * (1 + self.poissons_ratio))
        self.minimum_gage_thickness     = 0.5e-3 * Units.m
        self.minimum_width              = 25.4e-3 * Units.m
