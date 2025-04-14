# RCAIDE/Library/Attributes/Solids/Solid.py
#  
 
#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------

from RCAIDE.Framework.Core import Data

#-------------------------------------------------------------------------------
# Solid Data Class
#------------------------------------------------------------------------------- 
class Solid(Data):
    """
    Base class for all solid materials in RCAIDE. Provides fundamental mechanical and physical properties.

    Attributes
    ----------
    ultimate_tensile_strength : float
        Maximum tensile stress the material can withstand before failure in Pa
    ultimate_shear_strength : float
        Maximum shear stress the material can withstand before failure in Pa
    ultimate_bearing_strength : float
        Maximum bearing stress the material can withstand before failure in Pa
    yield_tensile_strength : float
        Tensile stress at which material begins to deform plastically in Pa
    yield_shear_strength : float
        Shear stress at which material begins to deform plastically in Pa
    yield_bearing_strength : float
        Bearing stress at which material begins to deform plastically in Pa
    minimum_gage_thickness : float
        Minimum manufacturable thickness in m
    density : float
        Material density in kg/m³

    Notes
    -----
    This class serves as the parent class for all specific material implementations.
    The default values are None and should be overridden by child classes with 
    specific material properties.

    **Definitions**
    
    'Ultimate Strength'
        The maximum stress that a material can withstand before failure
    
    'Yield Strength'
        The stress at which a material begins to deform plastically
    
    'Bearing Strength'
        The resistance of a material to crushing loads applied through a fastener or pin
    
    'Minimum Gage Thickness'
        The minimum thickness that can be reliably manufactured while maintaining 
        structural integrity
    """

    def __defaults__(self):
        """Default Instantiation of Physical Property Values
        
        Assumptions:
            None
        
        Source:
            None
        """

        self.ultimate_tensile_strength  = None
        self.ultimate_shear_strength    = None
        self.ultimate_bearing_strength  = None
        self.yield_tensile_strength     = None
        self.yield_shear_strength       = None
        self.yield_bearing_strength     = None
        self.minimum_gage_thickness     = None
        self.density                    = None
