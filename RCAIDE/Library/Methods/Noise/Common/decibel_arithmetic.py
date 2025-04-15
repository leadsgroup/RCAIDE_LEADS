# RCAIDE/Methods/Noise/Common/decibel_arithmetic.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# Python package imports   
import numpy as np  
    
# ----------------------------------------------------------------------------------------------------------------------  
#  pressure_ratio_to_SPL_arithmetic
# ----------------------------------------------------------------------------------------------------------------------       
def pressure_ratio_to_SPL_arithmetic(p_pref_total):
    ''' This computes the total SPL given mutiple acoustic pressure ratios 
    of one of mutiple sources
    
    Assumptions:
        None

    Source:
        None

    Inputs:
        Pressure Ratios       [unitless]

    Outputs: 
        Sound Pressure Level  [decibel]

    Properties Used:
        N/A 
    
    '''
    SPL_total = 10*np.log10(np.nansum(p_pref_total, axis = 3))
    return SPL_total


    
# ----------------------------------------------------------------------------------------------------------------------  
#  SPL_arithmetic
# ----------------------------------------------------------------------------------------------------------------------   
def SPL_arithmetic(SPL, sum_axis):
    """
    This function computes the total Sound Pressure Level (SPL) from multiple sources using decibel arithmetic.

    Parameters
    ----------
    SPL : array_like
        Sound Pressure Level [dB].
    sum_axis : int
        Axis along which the SPL values are summed.

    Returns
    -------
    SPL_total : float or array_like
        Total Sound Pressure Level [dB].

    Notes
    -----
    The function uses decibel arithmetic to sum SPL values from multiple sources. If the input SPL is one-dimensional, it returns the input as the total SPL.

    **Definitions**

    'SPL'
        Sound Pressure Level, a measure of the sound intensity.

    References
    ----------
    None
    """
    if SPL.ndim == 1:
        SPL_total = SPL 
    else:
        p_prefs   = 10**(SPL/10)
        SPL_total = 10*np.log10(np.nansum(p_prefs, axis = sum_axis))
        
    return SPL_total



# ----------------------------------------------------------------------------------------------------------------------
#  SPL_average
# ----------------------------------------------------------------------------------------------------------------------   
def SPL_average(SPL, avg_axis):
    '''This computes the average SPL from multiple azimuthal locations 
    using decibel arithmetic  
    
    Assumptions:
        None

    Source:
        None

    Inputs:
        SPL  -  Sound Pressure Level        [dB]

    Outputs: 
        SPL  -  Sound Pressure Level        [dB]
    
    Properties Used:
        N/A 
    
    '''
    if SPL.ndim == 1:
        SPL_total = SPL 
    else:
        p_prefs   = 10**(SPL/10)
        SPL_total = 10*np.log10(np.average(p_prefs, axis = avg_axis))
        
    return SPL_total

