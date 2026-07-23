import numpy as np
# RCAIDE/Methods/Aeroacoustics/Semi_Empirical/Engine/mixed_noise_component.py
# 
# 
# Created:  Jun 2026, M. Clarke , P. Siripun

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
from RCAIDE.Framework.Core import Units, Data

# Python package imports   
import numpy as np   
 
# ----------------------------------------------------------------------------------------------------------------------
#  Slat Noise Model 
# ----------------------------------------------------------------------------------------------------------------------

def slat_noise(R_val, phi, theta, Ls, gamma_s, sigma_s, alpha, segment, frequency, A=1e-5):
    """
    Computes the Slat Noise Power Spectral Density based on Guo (2010).
    
    Parameters:
    -----------
    velocity : float
        Mean flow velocity [ft/s].
    M : float
        Mach number [Unitless].
    phi : float
        Azimuthal angle [rad].
    theta : float
        Polar angle [rad].
    distance : float
        Distance from airplane to observer (r) [ft].
    frequency : array_like
        Frequency array (f) [Hz].
    Bs : float
        Slat span length [ft].
    Ls : float
        Slat chord length [ft].
    gamma_s : float
        Slat deployment angle [rad].
    g_s : float
        Slat gap in percent of wing chord [%].
    sigma_s : float
        Slat sweep angle [rad].
    alpha : float
        Aircraft wing angle of attack [rad].
    rho_0 : float
        Ambient fluid density [slugs/ft^3].
    c_0 : float
        Speed of sound [ft/s].
    A : float
        Empirical amplitude calibration constant.
        
    Returns:
    --------
    SPL : array_like
        Sound Pressure Level spectrum [dB] at the given frequencies.
    """
    #Unpack Segment Data:
    M = segment.state.conditions.freestream.mach_number
    distance = R_val[0][0]/ Units.feet
    rho_0 = segment.state.conditions.freestream.density / Units["slugs/ft^3"]
    c_0 = segment.state.conditions.freestream.speed_of_sound / Units["ft/s"]
    velocity = segment.state.conditions.freestream.velocity / Units["ft/s"]
    Ls = Ls/Units.ft

    U_eff = velocity * np.cos(sigma_s)
    M_eff = M * np.cos(sigma_s)
    
    # Strouhal Number Calculation
    St = (frequency * Ls) / U_eff
    
    #Spectral Shape Function F(St)
    St_peak = 2.0 
    F_St = (St / St_peak)**2 / ((1 + (St / St_peak)**2)**3.5)
    
    # Angle of Attack Correction for Local Mach Number
    local_accel_factor = 1.0 + 2.0 * np.sin(alpha)
    M_local = M_eff * local_accel_factor
    
    # Mach Number Dependence W(M) using the accelerated local flow
    W_M = M_local**4.5
    
    # Convective Amplification (Doppler Factor)
    if theta < (np.pi/2):
        doppler_factor = 1.0 - M * np.cos(theta)
    elif theta > (np.pi/2):
        doppler_factor = 1.0 + M * np.cos(theta)
    else:
        doppler_factor = 1.0
    convective_amplification = doppler_factor**(-2)
    
    # Directivity Function D(theta, phi) with Coordinate Rotation
    total_pitch = alpha + gamma_s
    
    # Transform the observer coordinates into the local slat coordinate system.
    cos_theta_local = (np.cos(theta) * np.cos(total_pitch) + 
                       np.sin(theta) * np.sin(phi) * np.sin(total_pitch))
    
    # Dipole directivity
    D_theta_phi = (1.0 - cos_theta_local**2) * np.cos(sigma_s)**2
    
    # Overall Scaling Factor
    ambient_scale = (rho_0 * c_0**2)**2
    spherical_spreading = 1/(distance**2)
    
    # Assemble Far-Field Noise Power Spectral Density (Pi)
    Pi = A * ambient_scale * W_M * spherical_spreading * convective_amplification * D_theta_phi * F_St
    
    # Convert to Sound Pressure Level (dB)
    p_ref_psf = (2*10**-5)/Units.psf #reference lowest spl
    SPL = 10.0 * np.log10(Pi / (p_ref_psf**2) + 1e-12) + + np.log10(0.231 * frequency)
    
    return SPL