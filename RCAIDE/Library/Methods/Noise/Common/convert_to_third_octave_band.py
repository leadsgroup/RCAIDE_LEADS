# RCAIDE/Methods/Noise/Common/convert_to_third_octave_band.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# Python package imports   
import numpy as np  
    
# ----------------------------------------------------------------------------------------------------------------------  
#  Convert to Third Octave Band 
# ----------------------------------------------------------------------------------------------------------------------       
def convert_to_third_octave_band(SPL,f,settings): 
    """This method converts the SPL spectrum from blade harmonic passing frequency
    to third octave spectrum
    
    Assumptions:
        N/A

    Source: 

    Inputs:
        SPL                    - sound pressure level                          [dB] 
        f                      - blade passing spectrum frequencies            [Hz]
        settings.    
            center_frequencies - center frequencies of the 1/3 octave spectrum [dB]  
            lower_frequencies  - lower frequencies of the 1/3 octave spectrum  [dB]
            upper_frequencies  - upper frequencies of the 1/3 octave spectrum  [dB]
        

    Outputs:
        SPL_third_octave       - SPL in the 1/3 octave spectrum                [dB] 

    Properties Used:
        N/A 
        
    """  
    # unpack 
    cf               = settings.center_frequencies
    lf               = settings.lower_frequencies
    uf               = settings.upper_frequencies 
    dim_cpt          = len(SPL[:,0,0])
    dim_mic          = len(SPL[0,:,0]) 
    num_cf           = len(cf)
    num_f            = len(f[0,0,:]) 
    
    uf_vals          = np.tile(uf[None,None,:,None],(dim_cpt,dim_mic,1,num_f))
    lf_vals          = np.tile(lf[None,None,:,None],(dim_cpt,dim_mic,1,num_f))
    f_vals           = np.tile(f[:,:,None,:],(1,1,num_cf,1))
    SPL_vals         = np.tile(SPL[:,:,None,:],(1,1,num_cf,1))
     
    upper_bool       = (f_vals  <= uf_vals)
    lower_bool       = (lf_vals <= f_vals)
    boolean          = np.logical_and(upper_bool,lower_bool)
    SPL_array        = boolean*SPL_vals
    p_prefs          = 10**(SPL_array/10)
    SPL_third_octave = 10*np.log10(np.sum(boolean*p_prefs, axis =3)) 
    SPL_third_octave[np.isinf(SPL_third_octave)]  = 0 
    
    return SPL_third_octave
