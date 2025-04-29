# RCAIDE/Methods/Noise/Correlation_Buildup/Airframe/clean_wing_noise.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
import numpy as np
from RCAIDE.Framework.Core import Units

# ----------------------------------------------------------------------------------------------------------------------  
# main and nose landing gear noise
# ----------------------------------------------------------------------------------------------------------------------  
def landing_gear_noise(D, H, wheels, M, velocity, phi, theta, distance, frequency):
    """
    This calculates the Landing gear 1/3 octave band sound pressure level and overall sound pressure level.

    Parameters
    ----------
    D : float
        Landing gear tyre diameter.
    H : float
        Landing gear strut length.
    wheels : int
        Number of wheels per unit.
    M : float
        Mach number.
    velocity : float
        Aircraft speed.
    phi : float
        Azimuthal angle [rad].
    theta : float
        Polar angle [rad].
    distance : float
        Distance from airplane to observer, evaluated at retarded time [ft].
    frequency : array_like
        Frequency array [Hz].

    Returns
    -------
    SPL : array_like
        Sound Pressure Level of the landing gear [dB].

    Notes
    -----
    The function uses correlation-based methods to compute the noise levels from the landing gear.

    **Definitions**

    'SPL'
        Sound Pressure Level, a measure of the sound intensity.

    References
    ----------
    Fink, Martin R. "Noise component method for airframe noise." Journal of aircraft 16.10 (1979): 659-665.
    """  
     
    velocity_kts = velocity/Units.knots

    if (wheels==1 or wheels==2):
        G1 =  130
        G2 = 10 *np.log10(4.5* ((frequency*D/velocity)**2) * (12.5 + ((frequency*D/velocity)**2) )**(-2.25) )   
    else:
        G1 = 123
        G2 = 10 *np.log10(0.3* ((frequency*D/velocity)**2) * (1  + 0.25*((frequency*D/velocity)**2) )**(-1.5) )    
 
    SPL   = 60.*np.log10(velocity_kts/194.0)+20.*np.log10(D/distance)+ G1 + G2 

    return SPL
