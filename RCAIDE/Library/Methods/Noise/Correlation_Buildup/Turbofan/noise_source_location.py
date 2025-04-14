# RCAIDE/Methods/Noise/Correlation_Buildup/Engine/noise_source_location.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports  
from RCAIDE.Framework.Core  import  Data 

# Python package imports   
import numpy as np   

# ----------------------------------------------------------------------------------------------------------------------     
#  Noise Source Location
# ----------------------------------------------------------------------------------------------------------------------       
def noise_source_location(B, Xo, zk, Diameter_primary, theta_p, Area_primary, Area_secondary, distance_microphone,
                          Diameter_secondary, theta, theta_s, theta_m, Diameter_mixed, Velocity_primary, Velocity_secondary,
                          Velocity_mixed, Velocity_aircraft, sound_ambient, Str_m, Str_s):
    """
    This function calculates the noise source location for primary, secondary, and mixed jet components.

    Parameters
    ----------
    B : array_like
        Array for storing intermediate calculations.
    Xo : float
        Initial position offset.
    zk : float
        Scaling factor for jet source location.
    Diameter_primary : float
        Diameter of the primary jet [m].
    theta_p : array_like
        Angles for the primary jet [rad].
    Area_primary : float
        Area of the primary jet [m^2].
    Area_secondary : float
        Area of the secondary jet [m^2].
    distance_microphone : float
        Distance from the noise source to the microphone [m].
    Diameter_secondary : float
        Diameter of the secondary jet [m].
    theta : float
        General angle used in calculations [rad].
    theta_s : array_like
        Angles for the secondary jet [rad].
    theta_m : array_like
        Angles for the mixed jet [rad].
    Diameter_mixed : float
        Diameter of the mixed jet [m].
    Velocity_primary : float
        Velocity of the primary jet [m/s].
    Velocity_secondary : float
        Velocity of the secondary jet [m/s].
    Velocity_mixed : float
        Velocity of the mixed jet [m/s].
    Velocity_aircraft : float
        Velocity of the aircraft [m/s].
    sound_ambient : float
        Ambient sound level [dB].
    Str_m : array_like
        Strouhal number for the mixed jet.
    Str_s : array_like
        Strouhal number for the secondary jet.

    Returns
    -------
    source_location : Data
        Contains the calculated angles for each jet component.
            - theta_p : array_like
                Angles for the primary jet [rad].
            - theta_s : array_like
                Angles for the secondary jet [rad].
            - theta_m : array_like
                Angles for the mixed jet [rad].

    Notes
    -----
    The function iteratively calculates the source location for each jet component by adjusting angles and positions based on geometric and velocity parameters.

    **Definitions**

    'theta_p', 'theta_s', 'theta_m'
        Angles for the primary, secondary, and mixed jet components, respectively.

    References
    ----------
    None
    """
    
    # P rimary jet source location
    XJ = np.zeros(24)
    
    for i in range(24):
        residual = Diameter_primary
        XJ[i]    = (zk*Diameter_primary)*(4.+4.*np.arctan((18.*theta_p[i]/np.pi)-9.)+(Area_secondary/Area_primary))
        B[i]     = (1./np.sin(theta))*(((Xo+XJ[i])/distance_microphone)+np.cos(theta))
        
        if (B[i]>=0.):
            theta_p[i]=np.arcsin(((B[i])**2.+1.)**(-0.5))
        else:
            theta_p[i]=np.pi-np.arcsin(((B[i])**2.+1.)**(-0.5))

        XJ[i] = (zk*Diameter_primary)*(4.+4.*np.arctan((18.*theta_p[i]/np.pi)-9.)+(Area_secondary/Area_primary))
        
        while residual>(Diameter_primary/200.):
            XJ_old = XJ[i]
            theta1 = theta_p[i]
            B[i]   = (1./np.sin(theta))*(((Xo+XJ[i])/distance_microphone)+np.cos(theta))
            
            if B[i]>=0.:
                theta_p[i] = np.arcsin(((B[i])**2.+1.)**(-0.5))
            else:
                theta_p[i] = np.pi-np.arcsin(((B[i])**2.+1.)**(-0.5))

            theta2     = theta_p[i]
            theta_p[i] = (theta1+theta2)/2.
            XJ[i]      = (zk*Diameter_primary)*(4.+4.*np.arctan((18.*theta_p[i]/np.pi)-9.)+(Area_secondary/Area_primary))
            residual   = np.abs(XJ_old-XJ[i])

        # Secondary jet source location
        residual = Diameter_secondary
        XJ_old   = 0.0
        XJ[i]    = (zk*Diameter_secondary)*(2.+1.6*np.arctan((4.5*theta_s[i]/np.pi)-2.25))*(1.+0.5/np.sqrt(Str_s[i])) \
            *  np.sqrt(1.+(0.7*Velocity_secondary/sound_ambient))*(Velocity_secondary/(Velocity_secondary-Velocity_aircraft))
        
        B[i]     = (1./np.sin(theta))*(((Xo+XJ[i])/distance_microphone)+np.cos(theta))
        
        if B[i]>=0.:
            theta_s[i] = np.arcsin(((B[i])**2.+1.)**(-0.5))
        else:
            theta_s[i] = np.pi-np.arcsin(((B[i])**2.+1.)**(-0.5))
            
        XJ[i] = (zk*Diameter_mixed)*(2.+1.6*np.arctan((4.5*theta_s[i]/np.pi)-2.25))*(1.+0.5/np.sqrt(Str_s[i]))* \
            np.sqrt(1.+(0.7*Velocity_secondary/sound_ambient))*(Velocity_secondary/(Velocity_secondary-Velocity_aircraft))
        
        while residual>(Diameter_mixed/200.):
            XJ_old = XJ[i]
            theta1 = theta_s[i]
            B[i]   = (1/np.sin(theta))*(((Xo+XJ[i])/distance_microphone)+np.cos(theta))
            
            if B[i]>=0.:
                theta_s[i] = np.arcsin((B[i]**2.+1.)**(-0.5))
            else:
                theta_s[i] = np.pi-np.arcsin(((B[i])**2.+1.)**(-0.5))

            theta2     = theta_s[i]
            theta_s[i] = (theta1+theta2)/2.
            XJ[i]      = (zk*Diameter_mixed)*(2.+1.6*np.arctan((4.5*theta_s[i]/np.pi)-2.25))*(1.+0.5/np.sqrt(Str_s[i]))* \
                np.sqrt(1.+(0.7*Velocity_secondary/sound_ambient))*(Velocity_secondary/(Velocity_secondary-Velocity_aircraft))
            
            residual = np.abs(XJ_old-XJ[i])

        #Mixed jet source location
        residual = Diameter_mixed
        XJ_old   = 0.
        XJ[i] = (zk*Diameter_mixed)*(3.+np.exp(-Str_m[i])+(2.+1.1*np.arctan((18.*theta_m[i]/np.pi)-13.))+ \
            (1.+0.5/np.sqrt(Str_m[i])))*np.sqrt(0.5+0.5*Velocity_mixed/sound_ambient) * \
            (Velocity_mixed/(Velocity_mixed-Velocity_aircraft))
        
        B[i] = (1./np.sin(theta))*(((Xo+XJ[i])/distance_microphone)+np.cos(theta))
        if B[i]>=0.:
            theta_m[i] = np.arcsin(((B[i])**2.+1.)**(-0.5))
        else:
            theta_m[i] = np.pi-np.arcsin(((B[i])**2.+1.)**(-0.5))

        XJ[i]=(zk*Diameter_mixed)*(3.+np.exp(-Str_m[i])+(2.+1.1*np.arctan((18.*theta_m[i]/np.pi)-13.))+\
            (1.+0.5/np.sqrt(Str_m[i])))*np.sqrt(0.5+0.5*Velocity_mixed/sound_ambient) \
            *(Velocity_mixed/(Velocity_mixed-Velocity_aircraft))
        
        while residual>(Diameter_mixed/200.):
            XJ_old = XJ[i]
            theta1 = theta_m[i]
            B[i]   = (1./np.sin(theta))*(((Xo+XJ[i])/distance_microphone)+np.cos(theta))
            
            if B[i]>=0.:
                theta_m[i] = np.arcsin(((B[i])**2.+1.)**(-0.5))
            else:
                theta_m[i] = np.pi-np.arcsin(((B[i])**2.+1.)**(-0.5))
                
            theta2     = theta_m[i]
            theta_m[i] = (theta1+theta2)/2.
            XJ[i]      = (zk*Diameter_mixed)*(3.+np.exp(-Str_m[i])+(2.+1.1*np.arctan((18.*theta_m[i]/np.pi)-13.))+\
                (1.+0.5/np.sqrt(Str_m[i])))*np.sqrt(0.5+0.5*Velocity_mixed/sound_ambient) \
                *(Velocity_mixed/(Velocity_mixed-Velocity_aircraft))
            
            residual   = abs(XJ_old-XJ[i])
   
   
    source_location = Data()
    source_location.theta_p = theta_p
    source_location.theta_s = theta_s
    source_location.theta_m = theta_m
    
    return source_location