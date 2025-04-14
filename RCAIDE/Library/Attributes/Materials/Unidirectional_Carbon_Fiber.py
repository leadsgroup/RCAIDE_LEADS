# Unidirectional_Carbon_Fiber.py
#
# Created: Jul, 2017, J. Smart
# Modified: Apr, 2018, J. Smart

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------

from .Solid import Solid
from RCAIDE.Framework.Core import Units

#-------------------------------------------------------------------------------
# Uni-Directional Carbon Fiber Solid Class
#-------------------------------------------------------------------------------

class Unidirectional_Carbon_Fiber(Solid):

    """
    A class representing unidirectional carbon fiber composite material properties.

    Attributes
    ----------
    ultimate_tensile_strength : float
        Maximum tensile stress before failure in Pa (1500e6)
    ultimate_shear_strength : float
        Maximum shear stress before failure in Pa (70e6)
    ultimate_bearing_strength : float
        Maximum bearing stress before failure in Pa (1500e6)
    yield_tensile_strength : float
        Stress at which material begins to deform plastically in Pa (1500e6)
    yield_shear_strength : float
        Shear stress at which material begins to deform plastically in Pa (70e6)
    yield_bearing_strength : float
        Bearing stress at which material begins to deform plastically in Pa (1500e6)
    minimum_gage_thickness : float
        Minimum manufacturable thickness in m (420e-6)
    density : float
        Material density in kg/m³ (1600)

    Notes
    -----
    This class implements material properties for unidirectional carbon fiber composites.
    The equal ultimate and yield strengths reflect the generally linear elastic behavior 
    of carbon fiber composites up to failure. Properties are based on median values from 
    manufacturer reported data.

    **Definitions**
    
    'Unidirectional Carbon Fiber'
        A composite material where all carbon fibers are aligned in a single direction,
        providing maximum strength along the fiber direction
    
    'Ultimate Strength'
        The maximum stress that a material can withstand before failure
    
    'Yield Strength'
        The stress at which a material begins to deform plastically (note: in composites,
        this is often equal to ultimate strength due to brittle failure behavior)

    References
    ----------
    [1] MatWeb. (n.d.). Overview of materials for Epoxy/Carbon Fiber Composite. Overview of materials for epoxy/carbon fiber composite. https://www.matweb.com/search/datasheet.aspx?matguid=39e40851fc164b6c9bda29d798bf3726 
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

        self.ultimate_tensile_strength  = 1500e6    * Units.Pa
        self.ultimate_shear_strength    = 70e6      * Units.Pa
        self.ultimate_bearing_strength  = 1500e6    * Units.Pa
        self.yield_tensile_strength     = 1500e6    * Units.Pa
        self.yield_shear_strength       = 70e6      * Units.Pa
        self.yield_bearing_strength     = 1500e6    * Units.Pa
        self.minimum_gage_thickness     = 420e-6    * Units.m
        self.density                    = 1600.     * Units['kg/(m**3)']
