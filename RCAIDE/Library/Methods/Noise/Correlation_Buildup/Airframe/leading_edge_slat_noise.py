# RCAIDE/Methods/Noise/Correlation_Buildup/Airframe/leading_edge_slat_noise.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
import numpy as np

from .clean_wing_noise import clean_wing_noise

# ----------------------------------------------------------------------------------------------------------------------  
#  slat leading edge noise
# ----------------------------------------------------------------------------------------------------------------------  
def leading_edge_slat_noise(SPL_wing,Sw,bw,velocity,viscosity,M,phi,theta,distance,frequency):
    """ This calculates the noise from the slat leading edge as a 1/3 octave band sound pressure level. 
    
     Assumptions:
         Correlation based.
         
     Inputs:
             SPL_wing                   - Sound Pressure Level of the clean wing                         [dB]
             Sw                         - Wing Area                                                      [sq.ft]
             bw                         - Wing Span                                                      [ft] 
             velocity                   - Aircraft speed                                                 [kts]
             viscosity                  - Dynamic viscosity                                              [kg m^-1s^-1]
             M                          - Mach number                                                    [unitless]
             phi                        - Azimuthal angle                                                [rad]
             theta                      - Polar angle                                                    [rad]
             distance                   - Distance from airplane to observer, evaluated at retarded time [ft]
             frequency                  - Frequency array                                                [Hz]
                                                                                                         
     Outputs: One Third Octave Band SPL                                                                  [dB]
         SPL                             - Sound Pressure Level of the slat leading edge                 [dB]
    
    Properties Used:
        None    
    """
     
    #Process
    SPLslat1   = SPL_wing+3.0
    SPLslat2   = clean_wing_noise(0.15*Sw,bw,1,1,velocity,viscosity,M,phi,theta,distance,frequency)
    peakfactor = 3+max(SPL_wing)-max(SPLslat2)
    SPLslat2   = SPLslat2+peakfactor

    SPL        = 10.*np.log10(10.0**(0.1*SPLslat1)+10.0**(0.1*SPLslat2))

    return SPL
