# RCAIDE/Methods/Noise/Metrics/A_weighting_metric.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
 
# Python package imports   
import numpy as np  
    
# ----------------------------------------------------------------------------------------------------------------------  
#  A_weighting_metric
# ----------------------------------------------------------------------------------------------------------------------      
def A_weighting_metric(SPL, f):
    """
    This method calculates the A-weighted Sound Pressure Level (SPL) given its spectra.

    Parameters
    ----------
    SPL : float or array_like
        Sound Pressure Level [dB].
    f : float or array_like
        Frequency of the sound [Hz].

    Returns
    -------
    SPL_dBA : float or array_like
        A-weighted Sound Pressure Level [dBA].

    Notes
    -----
    The function applies the A-weighting filter to the input SPL, which adjusts the levels to reflect the frequency sensitivity of human hearing.

    **Definitions**

    'SPL_dBA'
        A-weighted Sound Pressure Level, a measure of sound intensity adjusted for human hearing sensitivity.

    References
    ----------
    IEC 61672-1:2013 Electroacoustics - Sound level meters - Part 1: Specifications. IEC. 2013.
    """
    Ra_f = ((12194**2)*(f**4)) / (((f**2)+(20.6**2)) * ((f**2)+(12194**2)) * (((f**2) + 107.7**2)**0.5) * (((f**2)+ 737.9**2)**0.5))
    A_f = 2.0 + 20*np.log10(Ra_f)
    SPL_dBA = SPL + A_f
    return SPL_dBA
