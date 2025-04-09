# Epoxy.py
#
# Created: Jul, 2017, J. Smart
# ModifiedL Apr, 2018, J. Smart

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------

from .Solid import Solid
from RCAIDE.Framework.Core import Units

#-------------------------------------------------------------------------------
# Hardened Epoxy Resin Solid Class
#-------------------------------------------------------------------------------

class Epoxy(Solid):
    """ 
    A class representing hardened epoxy resin material properties.

    Attributes
    ----------
    ultimate_tensile_strength : float
        Maximum tensile stress before failure in Pa (0.0)
    ultimate_shear_strength : float
        Maximum shear stress before failure in Pa (0.0)
    ultimate_bearing_strength : float
        Maximum bearing stress before failure in Pa (0.0)
    yield_tensile_strength : float
        Stress at which material begins to deform plastically in Pa (0.0)
    yield_shear_strength : float
        Shear stress at which material begins to deform plastically in Pa (0.0)
    yield_bearing_strength : float
        Bearing stress at which material begins to deform plastically in Pa (0.0)
    minimum_gage_thickness : float
        Minimum manufacturable thickness in m (250e-6)
    density : float
        Material density in kg/m³ (1800)

    Notes
    -----
    This class implements material properties for cured epoxy resin. The zero values 
    for strength properties indicate that these should be specified based on the 
    specific epoxy formulation being used, as properties can vary significantly 
    between different epoxy systems.

    **Definitions**
    
    'Epoxy Resin'
        A thermosetting polymer that cures (hardens) when mixed with a catalyst
        or hardener
    
    'Ultimate Strength'
        The maximum stress that a material can withstand before failure
    
    'Yield Strength'
        The stress at which a material begins to deform plastically

    References
    ----------
    [1] MatWeb. (n.d.). Overview of materials for Epoxy, Cast, Unreinforced. Overview of materials for epoxy, cast, unreinforced. https://www.matweb.com/search/DataSheet.aspx?MatGUID=1c74545c91874b13a3e44f400cedfe39 
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

        self.ultimate_tensile_strength  = 0.0       * Units.Pa
        self.ultimate_shear_strength    = 0.0       * Units.Pa
        self.ultimate_bearing_strength  = 0.0       * Units.Pa
        self.yield_tensile_strength     = 0.0       * Units.Pa
        self.yield_shear_strength       = 0.0       * Units.Pa
        self.yield_bearing_strength     = 0.0       * Units.Pa
        self.minimum_gage_thickness     = 250e-6    * Units.m
        self.density                    = 1800.     * Units['kg/(m**3)']
