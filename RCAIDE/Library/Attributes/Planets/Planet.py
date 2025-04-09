# RCAIDE/Library/Attributes/Planets/Planet.py
# 
#
# Created:  Mar 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------  
#  Imports
# ----------------------------------------------------------------------------------------------------------------------  
 
from RCAIDE.Framework.Core import Data

# ----------------------------------------------------------------------------------------------------------------------  
#  Planet Class
# ----------------------------------------------------------------------------------------------------------------------   
class Planet(Data):
    """
    Base class for planetary bodies in RCAIDE. Provides fundamental planetary properties 
    and gravitational calculations.

    Attributes
    ----------
    mass : float
        Total mass of the planetary body in kg (defaults to 0.0)
    mean_radius : float
        Average radius of the planetary body in m (defaults to 0.0)

    Notes
    -----
    This class serves as the parent class for all specific planetary implementations.
    The default values are zero and should be overridden by child classes with 
    specific planetary properties.

    **Definitions**
    
    'Mean Radius'
        The average radius of a planetary body, accounting for polar flattening 
        and equatorial bulge
    
    'Mass'
        The total mass of the planetary body, including all components 
        (core, mantle, crust, atmosphere)

    See Also
    --------
    RCAIDE.Library.Attributes.Planets.Earth : Earth-specific implementation
    """
    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """           
        self.mass              = 0.0  # kg
        self.mean_radius       = 0.0  # m 
