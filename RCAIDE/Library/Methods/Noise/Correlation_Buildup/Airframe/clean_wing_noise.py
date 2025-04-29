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
# Clean Wing Noise
# ----------------------------------------------------------------------------------------------------------------------
def clean_wing_noise(S, b, ND, IsHorz, velocity, viscosity, M, phi, theta, distance, frequency):
    """
    This computes the 1/3 octave band sound pressure level and the overall sound pressure level from the clean wing.

    Parameters
    ----------
    S : float
        Wing area [sq.ft].
    b : float
        Wing span [ft].
    ND : int
        Constant from the method, set to 0 for clean wings and 1 for others.
    IsHorz : int
        Constant from the method, set to 1 for horizontal components.
    velocity : float
        Aircraft speed [kts].
    viscosity : float
        Dynamic viscosity.
    M : float
        Mach number.
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
        Sound Pressure Level of the clean wing [dB].

    Notes
    -----
    The function uses correlation-based methods to compute the noise levels from the clean wing.

    **Definitions**

    'SPL'
        Sound Pressure Level, a measure of the sound intensity.

    References
    ----------
    Fink, Martin R. "Noise component method for airframe noise." Journal of aircraft 16.10 (1979): 659-665.
    """
    distance_ft   = distance /Units.ft
    delta         = 0.37*(S/b)*(velocity*S/(b*viscosity))**(-0.2) 
    if IsHorz==1:
        DIR = np.cos(phi)
    elif IsHorz==0:
        DIR = np.sin(phi)

    if DIR==0:
        SPL = np.zeros(24)
    else:

        fmax      = 0.1*velocity / delta   # eqn 7   
        OASPL     = 50*np.log10((velocity*Units.ft/Units.kts)/100.0) + 10*np.log10(delta*Units.ft*b*Units.ft/((distance*Units.ft)**2.0)) * (DIR ** 2) * (np.cos(theta/2)) ** 2 + 101.3 
        SPL       = OASPL + 10.0*np.log10( 0.613* (frequency/fmax)**4 * ((frequency/fmax)**1.5 + 0.5)**(-4)) # eqn 5 
        Delta_SPL = -0.03* (distance_ft/500 ) * np.abs(((frequency/fmax)-1))**1.5 # eqn 6
        
        SPL += Delta_SPL
        
    return SPL
