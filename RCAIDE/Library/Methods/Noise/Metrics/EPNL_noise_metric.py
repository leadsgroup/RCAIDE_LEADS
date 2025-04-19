# RCAIDE/Methods/Noise/Metrics/EPNL_noise_metric.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# Python package imports   
import numpy as np  
    
# ----------------------------------------------------------------------------------------------------------------------  
#  EPNL_noise_metric
# ----------------------------------------------------------------------------------------------------------------------        
def EPNL_noise_metric(noise_data):
    """This method calculates the effective perceived noise level (EPNL) based on a
    time history Perceived Noise Level with Tone Correction (PNLT).
     
    Assumptions:
        N/A

    Source:
        N/A

    Inputs:
        PNLT - Perceived Noise Level with Tone Correction  [PNLdB]
     
     Outputs:
        EPNL - Effective Perceived Noise Level             [EPNdB]
     
    Properties Used:
        N/A  
    """
    PNLT = noise_data.PNLT
    
    # Maximum PNLT on the time history data    
    n_mic    = len(PNLT[0,:])
    PNLT_max = np.max(PNLT,axis=0)
    
    # Calculates the number of discrete points on the trajectory
    nsteps   = len(PNLT)    
    EPNL     = np.zeros(n_mic)
    for j in range(n_mic):
        # Finding the time duration for the noise history where PNL is higher than the maximum PNLT - 10 dB
        i = 0
        while PNLT[i][j]<=(PNLT_max[j]-10) and i<=nsteps:
            i = i+1
        t1 = i #t1 is the first time interval
        i  = i+1
    
        # Correction for PNLTM-10 when it falls outside the limit of the data
        if PNLT[nsteps-1][j]>=(PNLT_max[j]-10):
            t2=nsteps-2
        else:
            while i<=nsteps and PNLT[i][j]>=(PNLT_max[j]-10):
                i = i+1
            t2 = i-1 #t2 is the last time interval 
        
        # Calculates the integral of the PNLT which between t1 and t2 points
        sumation = 0
        for i in range (t1-1,t2+1):
            sumation = 10**(PNLT[i][j]/10)+sumation
            
        # Duration Correction calculation
        duration_correction = 10*np.log10(sumation)-PNLT_max[j]-13
                    
        # Final EPNL calculation
        EPNL[j] = PNLT_max[j]+duration_correction
    
    noise_data.EPNL = EPNL
    return   