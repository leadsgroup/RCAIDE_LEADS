# RCAIDE/Methods/Noise/Correlation_Buildup/Engine/jet_installation_effect.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
 
# Python package imports   
import numpy as np   

# ----------------------------------------------------------------------------------------------------------------------     
#  Jet Installation Effect
# ----------------------------------------------------------------------------------------------------------------------      
def jet_installation_effect(Xe, Ye, Ce, theta_s, Diameter_mixed):
    """
    This calculates the installation effect, in decibels, to be added to the predicted secondary jet noise level.

    Parameters
    ----------
    Xe : float
        Fan exit location downstream of the leading edge (Xe < Ce) [m].
    Ye : float
        Separation distance from the wing chord line to nozzle lip [m].
    Ce : float
        Wing chord length at the engine location [m].
    theta_s : float
        Angle for the secondary jet [rad].
    Diameter_mixed : float
        Diameter of the mixed jet [m].

    Returns
    -------
    INST_s : float
        Installation effect adjustment for the secondary jet noise level [dB].

    Notes
    -----
    The function calculates the installation effect based on geometric parameters and angles, assuming a significant impact on secondary jet noise levels.

    **Definitions**

    'INST_s'
        Installation effect, the adjustment in decibels due to the installation of the jet.

    References
    ----------
    [1] SAE ARP876D: Gas Turbine Jet Exhaust Noise Prediction (original)
    [2] de Almeida, Odenir. "Semi-empirical methods for coaxial jet noise prediction." (2008). (adapted)
    """
    # Installation effect
    INST_s = np.array([0.5 * ((Ce - Xe) ** 2 / (Ce * Diameter_mixed)) * 
                       (np.exp(-Ye / Diameter_mixed) * ((1.8 * theta_s / np.pi) - 0.6) ** 2)])
    INST_s[INST_s > 2.5] = 2.5

    return INST_s