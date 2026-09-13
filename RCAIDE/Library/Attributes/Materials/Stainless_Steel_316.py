# RCAIDE/Library/Attributes/Solids/Stainless_Steel_316.py
#
# Created:
#
#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from .Solid import Solid

#-------------------------------------------------------------------------------
# Stainless Steel 316
#-------------------------------------------------------------------------------
class Stainless_Steel_316(Solid):
    """
    A class representing stainless steel 316(L) material properties for aerospace
    and cryogenic applications.

    Attributes
    ----------
    density : float
        Material density in kg/m³ (8000).
    thermal_conductivity : float
        Heat conduction coefficient in W/(m·K) (≈0.9 at cryogenic temperatures).
    yield_tensile_strength : float
        Stress at which material begins to deform plastically in Pa (790e6).

    Notes
    -----
    Stainless steel 316(L) is an austenitic chromium-nickel-molybdenum alloy
    commonly used for cryogenic transfer lines and piping in preference to 304
    for its added molybdenum content, which improves resistance to pitting and
    crevice corrosion from condensation/purge cycling on externally-routed
    lines. Like 304, it retains strength and ductility at cryogenic
    temperatures and has low thermal conductivity relative to aluminum alloys.
    ``yield_tensile_strength`` uses a representative annealed 316L value at
    liquid-nitrogen temperature (77 K, ~790 MPa), the most commonly reported
    reference condition in the cryogenic materials literature, in place of the
    room-temperature value (~170 MPa) that would understate in-service
    strength for a line actually carrying cryogenic fuel.

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
        Uses representative values for annealed stainless steel 316L at
        cryogenic temperatures.
        """
        self.density                 = 8.0e3 * Units['kg/(m**3)']
        self.thermal_conductivity    = 0.9
        self.yield_tensile_strength  = 790e6 * Units.Pa
