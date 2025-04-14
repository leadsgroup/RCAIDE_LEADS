# RCAIDE/Methods/Noise/Correlation_Buildup/Engine/external_plug_effect.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
 
# Python package imports   
import numpy as np   

# ----------------------------------------------------------------------------------------------------------------------     
#  Ground Proximity Effect
# ----------------------------------------------------------------------------------------------------------------------         
def ground_proximity_effect(Velocity_mixed, sound_ambient, theta_m, engine_height, Diameter_mixed, frequency):
    """
    This function calculates the ground proximity effect, in decibels, for full-scale engine test stands.

    Parameters
    ----------
    Velocity_mixed : float
        Velocity of the mixed jet [m/s].
    sound_ambient : float
        Ambient sound level [SPL].
    theta_m : float
        Angle for the mixed jet [rad].
    engine_height : float
        Height of the engine above the ground [m].
    Diameter_mixed : float
        Diameter of the mixed jet [m].
    frequency : float
        Frequency of the sound wave [1/s].

    Returns
    -------
    GPROX_m : float
        Ground proximity effect adjustment for the mixed jet [dB].

    Notes
    -----
    The function assumes that the ground proximity effect is significant for the mixed jet component and calculates the noise adjustments accordingly.

    **Definitions**

    'GPROX_m'
        Ground Proximity Effect, the adjustment in decibels due to the proximity of the ground.

    References
    ----------
    [1] SAE ARP876D: Gas Turbine Jet Exhaust Noise Prediction (original)
    [2] de Almeida, Odenir. "Semi-empirical methods for coaxial jet noise prediction." (2008). (adapted)
    """ 
    # Ground proximity is applied only for the mixed jet component
    GPROX_m = (5*Velocity_mixed/sound_ambient)*np.exp(-(9*(theta_m/np.pi)-6.75)**2- \
        ((engine_height/Diameter_mixed)-2.5)**2)*(1+(np.sin((np.pi*engine_height*frequency/sound_ambient)-np.pi/2))**2)/ \
        (2+np.abs((engine_height*frequency/sound_ambient)-1))

    return GPROX_m