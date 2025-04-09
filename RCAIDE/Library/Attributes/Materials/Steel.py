# Steel.py
#
# Created: Jul, 2017, J. Smart
# Modified: Apr, 2018, J. Smart

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------

from .Solid import Solid
from RCAIDE.Framework.Core import Units

#-------------------------------------------------------------------------------
# AISI 4340 Steel Solid Class
#-------------------------------------------------------------------------------

class Steel(Solid):
    """ 
    A class representing AISI 4340 steel material properties.

    Attributes
    ----------
    ultimate_tensile_strength : float
        Maximum tensile stress before failure in Pa (1110e6)
    ultimate_shear_strength : float
        Maximum shear stress before failure in Pa (825e6)
    ultimate_bearing_strength : float
        Maximum bearing stress before failure in Pa (1110e6)
    yield_tensile_strength : float
        Stress at which material begins to deform plastically in Pa (710e6)
    yield_shear_strength : float
        Shear stress at which material begins to deform plastically in Pa (410e6)
    yield_bearing_strength : float
        Bearing stress at which material begins to deform plastically in Pa (710e6)
    minimum_gage_thickness : float
        Minimum manufacturable thickness in m (0.0)
    density : float
        Material density in kg/m³ (7850)

    Notes
    -----
    This class implements material properties for AISI 4340 steel, a high-strength 
    alloy steel commonly used in aerospace applications. The properties are based on 
    median values.

    **Definitions**
    
    'Ultimate Strength'
        The maximum stress that a material can withstand before failure
    
    'Yield Strength'
        The stress at which a material begins to deform plastically

    References
    ----------
    [1] MatWeb. (n.d.). Crucible Steel AISI 4340 Alloy Steel. Crucible Steel aisi 4340 alloy steel. https://www.matweb.com/search/DataSheet.aspx?MatGUID=32036e9a93d04975bec68fc8ca3a696d 
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

        self.ultimate_tensile_strength  = 1110e6    * Units.Pa
        self.ultimate_shear_strength    = 825e6     * Units.Pa
        self.ultimate_bearing_strength  = 1110e6    * Units.Pa
        self.yield_tensile_strength     = 710e6     * Units.Pa
        self.yield_shear_strength       = 410e6     * Units.Pa
        self.yield_bearing_strength     = 710e6     * Units.Pa
        self.minimum_gage_thickness     = 0.0       * Units.m
        self.density                    = 7850.     * Units['kg/(m**3)']
