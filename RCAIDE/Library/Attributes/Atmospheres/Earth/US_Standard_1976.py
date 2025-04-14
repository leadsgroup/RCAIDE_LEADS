# RCAIDE/Library/Attributes/Atmospheres/Earth/US_Standard_1976.py
# 
#
# Created:  Mar 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 
 
from RCAIDE.Framework.Core import Data , Units
from RCAIDE.Library.Attributes.Gases import Air
from RCAIDE.Library.Attributes.Atmospheres import Atmosphere
from RCAIDE.Library.Attributes.Planets import Earth 
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  US_Standard_1976 Class
# ----------------------------------------------------------------------------------------------------------------------   
class US_Standard_1976(Atmosphere):
    """
    Implementation of the 1976 U.S. Standard Atmosphere model.

    This class provides a complete implementation of the U.S. Standard Atmosphere (1976),
    defining atmospheric properties at various altitudes based on empirical data.

    Attributes
    ----------
    fluid_properties : Air
        Air properties object containing gas characteristics
    planet : Earth
        Earth properties object containing planetary parameters
    breaks : Data
        Container for atmospheric property breakpoints
            - altitude : array
                Geopotential altitude points in kilometers, ranging from -2 km to 84.852 km
            - temperature : array
                Temperature values at break points in Kelvin
            - pressure : array
                Pressure values at break points in Pascal
            - density : array
                Density values at break points in kg/m³

    Notes
    -----
    The U.S. Standard Atmosphere 1976 is a model defining atmospheric properties 
    up to 1000 km. This implementation covers the range from -2 km to 84.852 km, 
    which encompasses the primary region of interest for most aircraft operations.

    The model divides the atmosphere into layers with different temperature gradients
    and provides standardized values for Temperature, Pressure, Density, Speed of sound, and Viscosity.

    **Major Assumptions**
        * Hydrostatic equilibrium
        * Perfect gas behavior
        * Homogeneous composition below 80 km
        * Gravity variation with geometric height

    References
    ----------
    [1] NOAA, NASA, USAF, U.S. Standard Atmosphere, 1976 (1976). Retrieved December 30, 2024, from https://www.ngdc.noaa.gov/stp/space-weather/online-publications/miscellaneous/us-standard-atmosphere-1976/us-standard-atmosphere_st76-1562_noaa.pdf. 

    See Also
    --------
    RCAIDE.Library.Attributes.Atmospheres.Earth.Constant_Temperature : Simplified constant temperature model
    """
    
    def __defaults__(self):
        """This sets the default values at breaks in the atmosphere. 
    
        Assumptions:
            None
        
        Source:
            U.S. Standard Atmosphere (1976 version) 
        """
        self.tag = ' U.S. Standard Atmosphere (1976)'

        # break point data: 
        self.fluid_properties   = Air()
        self.planet             = Earth()
        self.breaks             = Data()
        self.breaks.altitude    = np.array( [-2.00    , 0.00,     11.00,      20.00,      32.00,      47.00,      51.00,      71.00,      84.852]) * Units.km     # m, geopotential altitude
        self.breaks.temperature = np.array( [301.15   , 288.15,   216.65,     216.65,     228.65,     270.65,     270.65,     214.65,     186.95])      # K
        self.breaks.pressure    = np.array( [127774.0 , 101325.0, 22632.1,    5474.89,    868.019,    110.906,    66.9389,    3.95642,    0.3734])      # Pa
        self.breaks.density     = np.array( [1.47808e0, 1.2250e0, 3.63918e-1, 8.80349e-2, 1.32250e-2, 1.42753e-3, 8.61606e-4, 6.42099e-5, 6.95792e-6])  # kg/m^3