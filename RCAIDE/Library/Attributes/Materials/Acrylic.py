# Acrylic.py
#
# Created: Jul, 2017, J. Smart
# Modified: Apr, 2018, J. Smart

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------

from .Solid import Solid
from RCAIDE.Framework.Core import Units

# ----------------------------------------------------------------------------------------------------------------------
# Acrylic Solid Class
# ----------------------------------------------------------------------------------------------------------------------
class Acrylic(Solid):

    """
    A class representing Polymethyl Methacrylate (PMMA/Acrylic) material properties.

    Attributes
    ----------
    ultimate_tensile_strength : float
        Maximum tensile stress the material can withstand before failure in Pa (75e6)
    ultimate_shear_strength : float
        Maximum shear stress the material can withstand before failure in Pa (55.2e6)
    ultimate_bearing_strength : float
        Maximum bearing stress the material can withstand before failure in Pa (0.0)
    yield_tensile_strength : float
        Stress at which material begins to deform plastically in Pa (75e6)
    yield_shear_strength : float
        Shear stress at which material begins to deform plastically in Pa (55.2e6)
    yield_bearing_strength : float
        Bearing stress at which material begins to deform plastically in Pa (0.0)
    minimum_gage_thickness : float
        Minimum manufacturable thickness in m (3.175e-3)
    density : float
        Material density in kg/m³ (1180)

    Notes
    -----
    All material properties are based on median values from manufacturer reported data.
    The bearing strength values are set to zero as they are typically not relevant for acrylic applications.

    **Definitions**
    
    'Ultimate Strength'
        The maximum stress that a material can withstand before failure
    
    'Yield Strength'
        The stress at which a material begins to deform plastically

    References
    ----------
    [1] MatWeb. (n.d.). Overview of materials for Acrylic, Extruded. Overview of materials for acrylic, extruded. https://www.matweb.com/search/DataSheet.aspx?MatGUID=632572aeef2a4224b5ac8fbd4f1b6f77 
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

        self.ultimate_tensile_strength  = 75e6      * Units.Pa
        self.ultimate_shear_strength    = 55.2e6    * Units.Pa
        self.ultimate_bearing_strength  = 0.0       * Units.Pa
        self.yield_tensile_strength     = 75e6      * Units.Pa
        self.yield_shear_strength       = 55.2e6    * Units.Pa
        self.yield_bearing_strength     = 0.0       * Units.Pa
        self.minimum_gage_thickness     = 3.175e-3  * Units.m
        self.density                    = 1180.     * Units['kg/(m**3)']
