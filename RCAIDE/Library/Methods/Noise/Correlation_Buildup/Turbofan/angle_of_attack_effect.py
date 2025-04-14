# RCAIDE/Methods/Noise/Correlation_Buildup/Engine/angle_of_attack_effect.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
 
# Python package imports   
import numpy as np   

# ----------------------------------------------------------------------------------------------------------------------     
#  Angle of Attack Effect
# ----------------------------------------------------------------------------------------------------------------------     
def angle_of_attack_effect(AoA,Mach_aircraft,theta_m):
    """
    This module predicts the free-field 1/3 Octave Band Sound Pressure Level (SPL) of coaxial subsonic jets for turbofan engines. It uses semi-empirical methods to account for various conditions such as flyover, static, and in-flight scenarios.
    
    See Also
    --------
    RCAIDE.Library.Methods.Noise.Metrics.A_weighting_metric : https://example.com/A_weighting_metric
    RCAIDE.Library.Methods.Noise.Common.SPL_arithmetic : https://example.com/SPL_arithmetic
    """

    # Angle of attack effect
    ATK_m = 0.5*AoA*Mach_aircraft*((1.8*theta_m/np.pi)-0.6)**2

    return ATK_m
