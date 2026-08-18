# Titanum.py
#
# Created: Jul, 2022, J. Smart
# Modified: Jul, 2026, S. Sharma

# -------------------------------------------------------------------------------
# Imports
# -------------------------------------------------------------------------------

from .Solid import Solid
from RCAIDE.Framework.Core import Units

# -------------------------------------------------------------------------------
# Titanium 
# ------------------------------------------------------------------------------- 
class Titanium(Solid):
    """
    A class representing Grade 5 Ti-6Al-4V titanium alloy material properties.

    Attributes
    ----------
    ultimate_tensile_strength : float
        Maximum tensile stress before failure in Pa (950e6)
    ultimate_shear_strength : float
        Maximum shear stress before failure in Pa (550e6)
    ultimate_bearing_strength : float
        Maximum bearing stress before failure in Pa (1860e6)
    yield_tensile_strength : float
        Stress at which material begins to deform plastically in Pa (880e6)
    yield_shear_strength : float
        Shear stress at which material begins to deform plastically in Pa (550e6)
    yield_bearing_strength : float
        Bearing stress at which material begins to deform plastically in Pa (1480e6)
    minimum_gage_thickness : float
        Minimum manufacturable thickness in m (0.0)
    density : float
        Material density in kg/m³ (4430)
    thermal_conductivity : float
        Thermal conductivity in W/(m*K) (6.7)

    Notes
    -----
    This class implements material properties for Ti-6Al-4V (Grade 5) titanium alloy 
    in the annealed condition. This alloy is widely used in aerospace applications 
    due to its excellent strength-to-weight ratio and corrosion resistance.

    **Definitions**
    
    'Ti-6Al-4V'
        A titanium alloy containing 6% aluminum and 4% vanadium, also known as Grade 5
    
    'Ultimate Strength'
        The maximum stress that a material can withstand before failure
    
    'Yield Strength'
        The stress at which a material begins to deform plastically

    References
    ----------
    [1] Aerospace Specification Metals, Inc. (n.d.). Titanium Ti-6Al-4V (Grade 5), Annealed. 
        ASM material data sheet. https://asm.matweb.com/search/SpecificMaterial.asp?bassnum=mtp641
    [2] HonTitan. Titanium Thermal Conductivity: Why It's Low and When That Matters. ~6.7 W/(m*K)
        at room temperature for Ti-6Al-4V, consistent with the 6.2-7.66 W/(m*K) experimental range
        reported for bulk Ti-6Al-4V. https://hontitan.com/titanium-thermal-conductivity/
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

        self.ultimate_tensile_strength  = 950e6  * Units.Pa
        self.ultimate_shear_strength    = 550e6  * Units.Pa
        self.ultimate_bearing_strength  = 1860e6 * Units.Pa
        self.yield_tensile_strength     = 880e6  * Units.Pa
        self.yield_shear_strength       = 550e6  * Units.Pa
        self.yield_bearing_strength     = 1480e6 * Units.Pa
        self.minimum_gage_thickness     = 0.0    * Units.m
        self.density                    = 4430.  * Units['kg/(m**3)']
        self.thermal_conductivity       = 6.7    # W/(m*K)
