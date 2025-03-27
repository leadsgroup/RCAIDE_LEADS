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
# Clean Wing Noise
# ----------------------------------------------------------------------------------------------------------------------
def clean_wing_noise(S,b,ND,IsHorz, velocity,viscosity,M,phi,theta,distance,frequency):
    """ This computes the 1/3 octave band sound pressure level and the overall sound pressure level from the clean wing,
    for a wing with area S (sq.ft) and span b (ft).  ND is a constant set to 0 for clean wings and set to 1 for propeller
    airplanes, jet transports with numerous large trailing edge flap tracks, flaps extended, or slats extended. ISHORZ must be set to 1.
    This function can be used for the horizontal tail by inserting the appropriate tail area and span. For a vertical tail, its appropriate
    area and height are used and ISHORZ must be set to 0.
    
    Assumptions:
        Correlation based.  
    
    Source:
       Fink, Martin R. "Noise component method for airframe noise." Journal of aircraft 16.10 (1979): 659-665. 
       
    Inputs:
            S                          - Wing Area  
            b                          - Wing Span  
            ND                         - Costant from the method
            IsHoriz                    - Costant from the method
            deltaw                     - Wing Turbulent Boundary Layer thickness [ft]
            velocity                   - Aircraft speed [kts]
            viscosity                  - Dynamic viscosity
            M                          - Mach number
            phi                        - Azimuthal angle [rad]
            theta                      - Polar angle [rad]
            distance                   - Distance from airplane to observer, evaluated at retarded time [ft]
            frequency                  - Frequency array [Hz] 


    Outputs: One Third Octave Band SPL [dB]
        SPL                              - Sound Pressure Level of the clean wing [dB]
        OASPL                            - Overall Sound Pressure Level of the clean wing [dB]

    Properties Used:
        None
    
    """ 
    distance_ft   = distance /Units.ft
    delta         = 0.37*(S/b)*(velocity*S/(b*viscosity))**(-0.2) 
    if IsHorz==1:
        DIR = np.cos(phi)
    elif IsHorz==0:
        DIR = np.sin(phi)

    if DIR==0:
        SPL = np.zeros(24)
    else:

        fmax      = 0.1*velocity / delta   # eqn 7   
        OASPL     = 50*np.log10((velocity*Units.ft/Units.kts)/100.0) + 10*np.log10(delta*Units.ft*b*Units.ft/((distance*Units.ft)**2.0)) * (DIR ** 2) * (np.cos(theta/2)) ** 2 + 101.3 
        SPL       = OASPL + 10.0*np.log10( 0.613* (frequency/fmax)**4 * ((frequency/fmax)**1.5 + 0.5)**(-4)) # eqn 5 
        Delta_SPL = -0.03* (distance_ft/500 ) * np.abs(((frequency/fmax)-1))**1.5 # eqn 6
        
        SPL += Delta_SPL
        
    return SPL
