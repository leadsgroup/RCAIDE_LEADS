import numpy as np

def calc_geometric_amplitude(params, is_high_freq, A0):
    """Calculates the geometric amplitude A_G (Equations 8.1 and 8.2)"""
    sigma_f = params['sigma_f']
    gamma_f = params['gamma_f']
    alpha = params['alpha']
    if not is_high_freq: 
        # LF Calculate curve according to paper
        return A0 * (1.0 + np.sin(sigma_f)) * (np.sin(gamma_f)**2)
    else:                
        # HF Calculate curve according to paper
        h = params['h']
        Lf = params['L_f']
        return A0 * (h / Lf) * (1.0 + np.sin(sigma_f)) * ((1.0 + np.sin(alpha))**2) * (np.sin(alpha + gamma_f)**4)

def calc_spectral_shape(f, M, l, c0, U, mu0, mu1, mu2):
    """Calculates the Spectral Shape Function F(f, M) Equation 5.17
    for reference, the terms:
        'h': Flap thickness (m)
        'L_f': Flap chord length (m)
        'alpha': Angle of attack (rad)
        'sigma_f': Flap sweep angle (rad)
        'gamma_f': Flap deployment angle (rad)
        'M': Flight Mach number
        'U': Flight velocity (m/s)
        'c0': Speed of sound (m/s)
        'rho0': Ambient density (kg/m^3)
        'r': Observer distance (m)
        'theta': Polar angle (overhead = 90 deg)"""
    k0 = (f * l) / c0 #k0 constant
    St = (f * l) / U #strouhals
    
    term1 = 1.0 + (mu0**2) * (St**2)
    term2 = 1.0 + (mu1**2) * ((1.0 + M)**2) * (St**2)
    term3 = 1.0 + (mu2**2) * (k0**2)
    
    return (k0**2) / (term1 * term2 * term3) #gives shape term as fx of tuned variables

def calc_mach_integral(M, mu0, mu1, mu2):
    """Calculates the Mach integral Equations 6.5 - 6.10."""
    sigma = [
        mu0 / M,
        mu1 * (1.0 + M) / M,
        mu2
    ]
    
    Gamma = [0.0, 0.0, 0.0]
    for n in range(3):
        denominator = 1.0
        for m in range(3):
            if m != n:
                denominator *= (sigma[n]**2 - sigma[m]**2)
        Gamma[n] = - (sigma[n]**2) / denominator
        
    k01 = 0.01 * M
    k02 = 100.0 * M
    
    I_M = 0.0
    for n in range(3): #for three of the terms
        arg = (sigma[n] * (k02 - k01)) / (1.0 + (sigma[n]**2) * k01 * k02)
        I_M += (Gamma[n] / sigma[n]) * np.arctan(arg)
        
    return I_M

def predict_flap_noise_spl(freq_array, params, constants):
    """
    Calculate the Sound Pressure Level spectrum.
        1D numpy array of SPL values (dB/Hz) matching the input frequencies.
    """
    if constants is None: #force input constant for matching, this is due to no data on the mius for 777 flaps
        constants = {
            'A0': 3, 'mu0': 7.693, 'mu1':10, 'mu2': 0.292, 'alpha_0': 0.005
        }
        
    p_ref = 2e-5 # Reference SPL in Pascals
    
    M = params['M']
    c0 = params['c0']
    U = params['U']
    r = params['r']
    theta = params['theta']
    rho0 = params['rho0']
    D = params.get('D', 1.0)# make this zero for now

    # Convective amplification / Doppler factor (Eq 4.4)
    Delta = 1.0 - M * np.cos(theta)
    
    # Doppler shifted source frequencies
    f_source = freq_array * Delta 
    
    # Calculate the Mach integral once for this specific Mach number
    I_M = calc_mach_integral(M, constants['mu0'], constants['mu1'], constants['mu2'])
    
    PSD_total = np.zeros_like(freq_array, dtype=float)
    comp_li = []
    # Loop over the two distinct noise bands to get the two bumps)
    for is_high_freq in [False, True]:
        # Switch characteristic lengths and power laws
        l = params['h'] if is_high_freq else params['L_f']
        n = 6 if is_high_freq else 5
        
        # Calculate functional components
        A_G = calc_geometric_amplitude(params, is_high_freq, constants['A0'])
        A_F = 1.0 # Standard assumption if flow is bundled into A0
        W_M = (M**n) / I_M
        if is_high_freq == False:
            f_source = f_source+10
        if is_high_freq == True:
            f_source = f_source+100

        F_f = calc_spectral_shape(f_source, M, l, c0, U, 
                                  constants['mu0'], constants['mu1'], constants['mu2'])

        shape_dB = 10.0 * np.log10(F_f)
        
        # Distance and absorption scaling
        length_scale = (params['L_f'] * l) / ((Delta**2) * (r**2))
        atmospheric_absorption = np.exp(-constants['alpha_0'] * r)
        # Combine all to get PSD
        PSD_component = (rho0**2) * (c0**4) * A_G * A_F * W_M * F_f * D * (l / c0) * length_scale * atmospheric_absorption
        PSD_total += PSD_component
        comp_li.append(shape_dB)
        
    # Convert total PSD [Pa^2/Hz] to SPL [dB/Hz]
    SPL = 10.0 * np.log10(PSD_total / (p_ref**2))
    
    return SPL,shape_dB,comp_li