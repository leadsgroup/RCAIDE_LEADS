# RCAIDE/Library/Attributes/Atmospheres/Earth/Constant_Temperature.py
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
# Constant_Temperature Class
# ----------------------------------------------------------------------------------------------------------------------      
class Constant_Temperature(Atmosphere):
    """
    Modified US Standard Atmosphere 1976 model with constant temperature profile.

    This class implements a simplified atmospheric model where temperature remains constant
    across all altitudes while maintaining US Standard 1976 pressure and density variations.

    Attributes
    ----------
    fluid_properties : Air
        Air properties object containing gas characteristics
    planet : Earth
        Earth properties object containing planetary parameters
    breaks : Data
        Container for atmospheric property breakpoints
            - altitude : array
                Geopotential altitude points in meters
            - temperature : array
                Temperature values at break points (constant 301.15 K)
            - pressure : array
                Pressure values at break points in Pascal
            - density : array
                Density values at break points in kg/m³

    Notes
    -----
    This model is useful for simplified analyses where temperature variations
    with altitude are not critical, while still maintaining realistic pressure
    and density gradients.

    **Major Assumptions**
        * Temperature remains constant at 301.15 K across all altitudes
        * Pressure and density variations follow US Standard 1976 model
        * Valid from -2 km to 84.852 km geopotential altitude

    References
    ----------
    [1] NOAA, NASA, USAF, U.S. Standard Atmosphere, 1976 (1976). Retrieved December 30, 2024, from https://www.ngdc.noaa.gov/stp/space-weather/online-publications/miscellaneous/us-standard-atmosphere-1976/us-standard-atmosphere_st76-1562_noaa.pdf. 

    See Also
    --------
    RCAIDE.Library.Attributes.Atmospheres.Earth.US_Standard_1976 : Full US Standard Atmosphere implementation
    """
    def __defaults__(self):
        """This sets the default values at breaks in the atmosphere.

        Assumptions:
            Constant temperature

        Source:
            U.S. Standard Atmosphere (1976 version)

        Inputs:
        None

        Outputs:
        None

        Properties Used:
        None
        """          
        self.fluid_properties = Air()
        self.planet = Earth()
        self.breaks = Data()
        self.breaks.altitude    = np.array( [-2.00    , 0.00,     11.00,      20.00,      32.00,      47.00,      51.00,      71.00,      84.852]) * Units.km     # m, geopotential altitude
        self.breaks.temperature = np.array( [301.15   , 301.15,    301.15,    301.15,     301.15,     301.15,     301.15,     301.15,     301.15])      # K
        self.breaks.pressure    = np.array( [127774.0 , 101325.0, 22632.1,    5474.89,    868.019,    110.906,    66.9389,    3.95642,    0.3734])      # Pa
        self.breaks.density     = np.array( [1.545586 , 1.2256523,.273764,	 .0662256,	0.0105000 ,	1.3415E-03,	8.0971E-04,	4.78579E-05, 4.51674E-06]) #kg/m^3 
    
    pass
