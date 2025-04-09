# Bidirectional_Carbon_Fiber.py
#
# Created: Jul, 2017, J. Smart
# Modified: Apr, 2018, J. Smart

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------

from .Solid import Solid
from RCAIDE.Framework.Core import Units

#-------------------------------------------------------------------------------
# Bi-Directional Carbon Fiber Solid Class
#-------------------------------------------------------------------------------

class Bidirectional_Carbon_Fiber(Solid):
    """ 
    A class representing bidirectional carbon fiber composite material properties.

    Attributes
    ----------
    ultimate_tensile_strength : float
        Maximum tensile stress before failure in Pa (600e6)
    ultimate_shear_strength : float
        Maximum shear stress before failure in Pa (90e6)
    ultimate_bearing_strength : float
        Maximum bearing stress before failure in Pa (600e6)
    yield_tensile_strength : float
        Stress at which material begins to deform plastically in Pa (600e6)
    yield_shear_strength : float
        Shear stress at which material begins to deform plastically in Pa (90e6)
    yield_bearing_strength : float
        Bearing stress at which material begins to deform plastically in Pa (600e6)
    minimum_gage_thickness : float
        Minimum manufacturable thickness in m (420e-6)
    density : float
        Material density in kg/m³ (1600)

    Notes
    -----
    This class implements material properties for bidirectional woven carbon fiber composites.
    Properties are based on median values from manufacturer reported data.
    The equal ultimate and yield strengths reflect the generally linear elastic behavior 
    of carbon fiber composites up to failure.

    **Definitions**
    
    'Bidirectional Carbon Fiber'
        A composite material with carbon fibers woven in two perpendicular directions,
        providing similar properties in both directions
    
    'Ultimate Strength'
        The maximum stress that a material can withstand before failure
    
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

        self.ultimate_tensile_strength  = 600e6     * Units.Pa
        self.ultimate_shear_strength    = 90e6      * Units.Pa
        self.ultimate_bearing_strength  = 600e6     * Units.Pa
        self.yield_tensile_strength     = 600e6     * Units.Pa
        self.yield_shear_strength       = 90e6      * Units.Pa
        self.yield_bearing_strength     = 600e6     * Units.Pa
        self.minimum_gage_thickness     = 420e-6    * Units.m
        self.density                    = 1600.     * Units['kg/(m**3)']
