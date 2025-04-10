# RCAIDE/Library/Attributes/Planets/Earth.py
# 
#
# Created:  Mar 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------  
#  Imports
# ----------------------------------------------------------------------------------------------------------------------  
 
from .Planet                import Planet 

# ----------------------------------------------------------------------------------------------------------------------  
#  Earth Class
# ----------------------------------------------------------------------------------------------------------------------  
class Earth(Planet):
    """
    A class representing Earth's physical properties and gravitational characteristics.

    Attributes
    ----------
    tag : str
        Identifier for the planet ('Earth')
    mass : float
        Mass of Earth in kg (5.98e24)
    mean_radius : float
        Average radius of Earth in m (6.371e6)
    sea_level_gravity : float
        Gravitational acceleration at sea level in m/s² (9.80665)

    Notes
    -----
    This class implements standard Earth properties used in aerospace calculations.
    Values are based on internationally accepted standards for Earth's physical parameters.
    """
    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """         
        self.tag = 'Earth'
        self.mass              = 5.98e24  # kg
        self.mean_radius       = 6.371e6  # m
        self.sea_level_gravity = 9.80665  # m/s^2    

    def compute_gravity(self, H=0.0):
        """
        Compute the gravitational acceleration at a given altitude above Earth's surface.

        Parameters
        ----------
        H : float, optional
            Altitude above sea level in meters. Default is 0.0

        Returns
        -------
        gh : float
            Gravitational acceleration at the specified altitude in m/s²

        Notes
        -----
        **Theory**
        Uses the inverse square law for gravitational acceleration:
        .. math::
            g(h) = g_0 \\left(\\frac{R_e}{R_e + h}\\right)^2

        where:
            - g₀ is sea level gravity
            - Rₑ is Earth's mean radius
            - h is altitude above sea level
        """          
        # Unpack
        g0  = self.sea_level_gravity
        Re  = self.mean_radius 
        
        # Calculate gravity
        gh = g0*(Re/(Re+H))**2.0
        
        return gh