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
# main and nose landing gear noise
# ----------------------------------------------------------------------------------------------------------------------  
def landing_gear_noise(D,H,wheels,M,velocity,phi,theta,distance,frequency):
    """This calculates the Landing gear 1/3 octave band sound pressure level and overall sound pressure level
    for a tyre diameter D, a strut length H and WHEELS number of  wheels per unit.

    Assumptions:
        Correlation based.

    Source:
       Fink, Martin R. "Noise component method for airframe noise." Journal of aircraft 16.10 (1979): 659-665.


    Inputs:
        D         - Landing gear tyre diameter                                      
        H         - Lading gear strut length                                        
        wheels    - Number of wheels per unit                                      [-]
        M         - Mach number                                                    [-]
        velocity  - Aircraft speed                                                  
        phi       - Azimuthal angle                                                [rad]
        theta     - Polar angle                                                    [rad]
        distance  - Distance from airplane to observer, evaluated at retarded time [ft]
        frequemcy - Frequency array                                                [Hz] 

    Outputs: One Third Octave Band SPL [dB]
        SPL           - Sound Pressure Level of the landing gear         [dB]
        OASPL         - Overall Sound Pressure Level of the landing gear [dB] 
 
    """  
     
    velocity_kts = velocity/Units.knots

    if (wheels==1 or wheels==2):
        G1 =  130
        G2 = 10 *np.log10(4.5* ((frequency*D/velocity)**2) * (12.5 + ((frequency*D/velocity)**2) )**(-2.25) )   
    else:
        G1 = 123
        G2 = 10 *np.log10(0.3* ((frequency*D/velocity)**2) * (1  + 0.25*((frequency*D/velocity)**2) )**(-1.5) )    
 
    SPL   = 60.*np.log10(velocity_kts/194.0)+20.*np.log10(D/distance)+ G1 + G2 

    return SPL
