# RCAIDE/Library/Methods/Thermal_Management/Heat_Exchangers/Cross_Flow_Heat_Exchanger/compute_heat_exhanger_factors.py
# 
# Created : Dec 2022, C.R. Zhao
# Modified: Jun 2023, M. Clarke

import numpy as np
from scipy import interpolate

# ----------------------------------------------------------------------
#  Methods
# ---------------------------------------------------------------------- 
def compute_heat_exhanger_factors(kc_vals,ke_vals,delta, Re):
    '''
    Core library about Kc & Ke factors for different types of HEX"
    
     Inputs:  
             kc_vals
             ke_vals
             Re
              
          Outputs:   
             Kc
             Ke
              
          Assumptions: 
             N/A 
        
          Source:
             None
    
    '''
    Kc = np.zeros(shape=Re.shape)
    for i in range(Kc.shape[0]):
        if Re[i] < 2000:
            Kc[i] = -0.4252 * delta * delta + 0.0326 * delta + 1.1798
        elif Re[i] > 1e4:
            Kc[i] = -0.419 * delta * delta + 0.0208 * delta + 0.4033
        else:
            delta_set = kc_vals[:, 0]
            Re_set = [2e3, 3e3, 5e3, 1e4]
            Kc_set_T = np.array([(kc_vals[:, 2]), (kc_vals[:, 3]), (kc_vals[:, 4]), (kc_vals[:, 5])])
            f = interpolate.interp2d(delta_set, Re_set, Kc_set_T, kind='linear')
            Kc[i] = f(delta, Re[i])
    
    Ke = np.zeros(shape=Re.shape)
    for i in range(Ke.shape[0]):
        if Re[i] < 2000:
            Ke[i] = 0.996 * delta * delta - 2.7687 * delta + 1.0016
        elif Re[i] > 1e4:
            Ke[i] = 0.9832 * delta * delta - 1.9823 * delta + 1
        else:
            delta_set = ke_vals[:, 0]
            Re_set = [2e3, 3e3, 5e3, 1e4]
            Ke_set_T = np.array([(ke_vals[:, 2]), (ke_vals[:, 3]), (ke_vals[:, 4]), (ke_vals[:, 5])])
            f = interpolate.interp2d(delta_set, Re_set, Ke_set_T, kind='linear')
            Ke[i] = f(delta, Re[i])
    return Kc, Ke