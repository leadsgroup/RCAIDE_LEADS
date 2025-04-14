# Paint.py
#
# Created: Jul, 2017, J. Smart
# Modified: Apr, 2018, J. Smart

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------

from .Solid import Solid
from RCAIDE.Framework.Core import Units

#-------------------------------------------------------------------------------
# Paint and/or Vinyl Surface Convering Solid Class
#-------------------------------------------------------------------------------

class Paint(Solid):
    """
    A class representing paint and vinyl surface coating material properties.

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
        Minimum applicable thickness in m (150e-6)
    density : float
        Material density in kg/m³ (1800)

    Notes
    -----
    This class implements material properties for paint and vinyl surface coatings. 
    The zero values for strength properties reflect that these coatings are non 
    load-bearing elements. The minimum gage thickness represents a typical 
    coating thickness for aerospace applications.

    **Definitions**
    
    'Surface Coating'
        A material applied to the surface of another material to provide protection, 
        decoration, or other functional properties
    
    'Minimum Gage Thickness'
        The minimum thickness required for adequate surface coverage and protection
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
        self.minimum_gage_thickness     = 150e-6    * Units.m
        self.density                    = 1800.     * Units['kg/(m**3)']
