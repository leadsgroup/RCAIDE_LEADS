# RCAIDE/Methods/Noise/Correlation_Buildup/Engine/secondary_noise_component.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
 
# Python package imports   
import numpy as np   

# ----------------------------------------------------------------------------------------------------------------------     
#  Secondary Noise Component
# ----------------------------------------------------------------------------------------------------------------------       
def secondary_noise_component(Velocity_primary, theta_s, sound_ambient, Velocity_secondary, Velocity_aircraft,
                              Area_primary, Area_secondary, DSPL_s, EX_s, Str_s):
    """
    This function calculates the noise contribution of the secondary jet component.

    Parameters
    ----------
    Velocity_primary : float
        Velocity of the primary jet [m/s].
    theta_s : float
        Angle for the secondary jet [rad].
    sound_ambient : float
        Ambient sound level [dB].
    Velocity_secondary : float
        Velocity of the secondary jet [m/s].
    Velocity_aircraft : float
        Velocity of the aircraft [m/s].
    Area_primary : float
        Area of the primary jet [m^2].
    Area_secondary : float
        Area of the secondary jet [m^2].
    DSPL_s : float
        Decibel Sound Pressure Level for the secondary jet [dB].
    EX_s : float
        Excess noise level for the secondary jet.
    Str_s : float
        Strouhal number for the secondary jet.

    Returns
    -------
    SPL_s : float
        Sound Pressure Level for the secondary jet component [dB].

    Notes
    -----
    The function uses semi-empirical methods to calculate the noise contribution of the secondary jet component.

    **Definitions**

    'SPL_s'
        Sound Pressure Level for the secondary jet component.

    References
    ----------
    [1] SAE ARP876D: Gas Turbine Jet Exhaust Noise Prediction (original)
    [2] de Almeida, Odenir. "Semi-empirical methods for coaxial jet noise prediction." (2008). (adapted)
    """

    # Calculation of the velocity exponent
    velocity_exponent = 0.5 * 0.1*theta_s

    # Calculation of the Source Strengh Function (FV)
    FV = ((Velocity_secondary-Velocity_aircraft)/sound_ambient)**velocity_exponent *\
        ((Velocity_secondary+Velocity_aircraft)/sound_ambient)**(1-velocity_exponent)

    # Determination of the noise model coefficients
    Z1 = -18*((1.8*theta_s/np.pi)-0.6)**2
    Z2 = -14-8*((1.8*theta_s/np.pi)-0.6)**3
    Z3 = -0.7
    Z4 = 0.6 - 0.5*((1.8*theta_s/np.pi)-0.6)**2+0.5*(0.6-np.log10(1+Area_secondary/Area_primary))
    Z5 = 51 + 54*theta_s/np.pi - 9*((1.8*theta_s/np.pi)-0.6)**3
    Z6 = 99 + 36*theta_s/np.pi - 15*((1.8*theta_s/np.pi)-0.6)**4 + \
        5*Velocity_secondary*(Velocity_primary-Velocity_secondary)/(sound_ambient**2) +  DSPL_s + EX_s

    # Determination of Sound Pressure Level for the secondary jet component
    SPL_s = (Z1*np.log10(FV)+Z2)*(np.log10(Str_s)-Z3*np.log10(FV)-Z4)**2 + Z5*np.log10(FV) + Z6

    return SPL_s 

