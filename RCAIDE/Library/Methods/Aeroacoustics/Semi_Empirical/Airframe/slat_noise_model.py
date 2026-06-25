import numpy as np

def slat_noise(velocity, M, phi, theta, distance, frequency, 
                Ls, rho_0, c_0, segment, A=1e-5):
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
    
    # 1. Strouhal Number Calculation
    St = (frequency * Ls) / velocity
    
    # 2. Spectral Shape Function F(St)
    St_peak = 2.0  # Assumed peak Strouhal number for the cove flow resonance
    F_St = (St / St_peak)**2 / ((1 + (St / St_peak)**2)**3.5)
    
    # 3. Mach Number Dependence W(M) reference paper
    W_M = M**4.5
    
    # 4. Convective Amplification (Doppler Factor)
    # Lambda = 1 - M * cos(theta) for source moving towards observer
    if theta < (np.pi/2):
        doppler_factor = 1.0 - M * np.cos(theta)
    elif theta > (np.pi/2):
        doppler_factor = 1.0 + M * np.cos(theta)
    else:
        doppler_factor = 1.0
    convective_amplification = doppler_factor**(-2)
    
    # 5. Directivity Function D(theta, phi)
    # Simplified dipole directivity:
    D_theta_phi = np.sin(theta)**2 * np.cos(phi)**2
    
    # 6. Overall Scaling Factor (Ambient Medium & Distance)
    # (rho_0 * c_0^2)^2 is the ambient medium pressure scaling squared
    ambient_scale = (rho_0 * c_0**2)**2
    spherical_spreading = distance**(-2)
    
    # 7. Assemble Far-Field Noise Power Spectral Density (Pi)
    Pi = A * ambient_scale * W_M * spherical_spreading * convective_amplification * D_theta_phi * F_St
    
    # Convert Power Spectral Density to Sound Pressure Level (dB)
    # Reference pressure in air is typically 20e-6 Pa (converted to psf if using Imperial)
    p_ref_psf = segment.conditions.freestream.pressure*0.0208854 # pascals converted to lb/sf
    SPL = 10.0 * np.log10(Pi / (p_ref_psf**2) + 1e-12)
    
    return SPL