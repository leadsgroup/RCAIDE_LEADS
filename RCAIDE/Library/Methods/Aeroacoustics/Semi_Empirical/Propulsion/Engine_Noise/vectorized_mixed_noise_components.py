# RCAIDE/Methods/Aeroacoustics/Semi_Empirical/Engine/mixed_noise_component.py
# 
# 
# Created:  Jun 2026, M. Clarke, P. Siripun
# Modified: Vectorized for multi-receptor array broadcasting

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
from RCAIDE.Framework.Core import Units, Data
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.interpolate_fan_noise import get_spl_fan, create_interpolator_fan
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.interpolate_core_noise import get_spl, create_interpolator
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan import design_turbofan
from RCAIDE.Library.Methods.Aeroacoustics.Common import SPL_arithmetic 
from RCAIDE.Library.Methods.Aeroacoustics.Metrics import A_weighting_metric

# Python package imports   
import numpy as np   
import math

# ----------------------------------------------------------------------------------------------------------------------
#  Safe Interpolator Wrappers (Ensures Vectorization doesn't break RCAIDE strict interpolators)
# ----------------------------------------------------------------------------------------------------------------------
def _safe_interp_1d(interp_func, interp_obj, name, x):
    """Safely evaluates a 1D interpolator over an N-D array."""
    try:
        res = interp_func(interp_obj, name, x)
        if isinstance(res, np.ndarray) and res.shape == x.shape:
            return res
    except:
        pass
    # Fallback if the specific RCAIDE interpolator rejects arrays
    res = np.zeros_like(x, dtype=float)
    for idx, val in np.ndenumerate(x):
        res[idx] = interp_func(interp_obj, name, val)
    return res

def _safe_interp_2d(interp_func, interp_obj, x, y):
    """Safely evaluates a 2D interpolator over broadcasted N-D arrays."""
    X, Y = np.broadcast_arrays(x, y)
    try:
        res = interp_func(interp_obj, X, Y)
        if isinstance(res, np.ndarray) and res.shape == X.shape:
            return res
    except:
        pass
    # Fallback
    res = np.zeros_like(X, dtype=float)
    for idx in np.ndindex(X.shape):
        res[idx] = interp_func(interp_obj, X[idx], Y[idx])
    return res

# ----------------------------------------------------------------------------------------------------------------------
#  Flap Noise Model 
# ----------------------------------------------------------------------------------------------------------------------
def flap_noise_model(R_val, theta_flap, cf, thickness, deltaf, frequency, segment):
    
    # Vectorize inputs for broadcasting: R_val/theta as column vectors (N_mic, 1), freq as row (1, N_freq)
    r = np.atleast_1d(R_val).reshape(-1, 1)
    theta = np.atleast_1d(theta_flap).reshape(-1, 1)
    freq = np.atleast_1d(frequency).reshape(1, -1)
    
    params = Data(
        h = thickness,                  
        L_f = cf,                       
        alpha = segment.state.conditions.aerodynamics.angles.alpha,                          
        sigma_f = 0.436332,                 
        gamma_f = deltaf,                                                              
        M = segment.state.conditions.freestream.mach_number,                                 
        U = segment.state.conditions.freestream.velocity,                                    
        c0 = segment.state.conditions.freestream.speed_of_sound,                             
        rho0 = segment.state.conditions.freestream.density,                                  
        r = r,                                                                           
        theta = theta 
    )

    constants = Data(
        A0= 3e-5, mu0= 0.7693, mu1= 1.0, mu2= 0.292, alpha_0= 0.008
    )
        
    p_ref = 2e-5 
    M = params.M
    c0 = params.c0
    U = params.U
    rho0 = params.rho0

    # Convective amplification / Doppler factor
    Delta = 1.0 - M * np.cos(theta)
    
    # Calculate the Mach integral once
    I_M = calc_mach_integral(M, constants.mu0, constants.mu1, constants.mu2)
    
    # Initialize total PSD array with correct broadcasted shape (N_mic, N_freq)
    PSD_total = np.zeros(np.broadcast(theta, freq).shape, dtype=float)
    
    for is_high_freq in [False, True]:
        l = params.h if is_high_freq else params.L_f
        n = 6 if is_high_freq else 5
        
        A_G = calc_geometric_amplitude(params, is_high_freq, constants.A0)
        A_F = 1.0 
        W_M = (M**n) / I_M
        
        f_source = freq / Delta
        F_f = calc_spectral_shape(f_source, M, l, c0, U, constants.mu0, constants.mu1, constants.mu2)
        
        length_scale = (params.L_f * l) / ((Delta**2) * (r**2))
        atmospheric_absorption = np.exp(-constants.alpha_0 * r)
        
        PSD_component = (rho0**2) * (c0**4) * A_G * A_F * W_M * F_f * (l / c0) * length_scale * atmospheric_absorption
        PSD_total += PSD_component
        
    # Convert PSD [Pa^2/Hz] to 1/3 Octave Band SPL [dB]
    SPL = 10.0 * np.log10(PSD_total / (p_ref**2) + 1e-12) + 10.0 * np.log10(0.231 * freq)
    return SPL 

def calc_geometric_amplitude(params, is_high_freq, A0):
    sigma_f = params.sigma_f
    gamma_f = params.gamma_f
    alpha = params.alpha
    if not is_high_freq: 
        return A0 * (1.0 + np.sin(sigma_f)) * (np.sin(gamma_f)**2)
    else:                
        h = params.h
        Lf = params.L_f
        return A0 * (h / Lf) * (1.0 + np.sin(sigma_f)) * ((1.0 + np.sin(alpha))**2) * (np.sin(alpha + gamma_f)**4)

def calc_spectral_shape(f, M, l, c0, U, mu0, mu1, mu2):
    omega = 2.0 * np.pi * f  # Converted to angular frequency
    k0 = (omega * l) / c0 
    St = (omega * l) / U 
    
    term1 = 1.0 + (mu0**2) * (St**2)
    term2 = 1.0 + (mu1**2) * ((1.0 + M)**2) * (St**2)
    term3 = 1.0 + (mu2**2) * (k0**2)
    
    return (k0**2) / (term1 * term2 * term3) 

def calc_mach_integral(M, mu0, mu1, mu2):
    sigma = [mu0 / M, mu1 * (1.0 + M) / M, mu2]
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
    for n in range(3): 
        arg = (sigma[n] * (k02 - k01)) / (1.0 + (sigma[n]**2) * k01 * k02)
        I_M += (Gamma[n] / sigma[n]) * np.arctan(arg)
        
    return I_M

# ----------------------------------------------------------------------------------------------------------------------
#  Slat Noise Model 
# ----------------------------------------------------------------------------------------------------------------------
def slat_noise(R_val, phi, theta_raw, Ls, gamma_s, sigma_s, alpha, segment, frequency, A):
    
    distance = np.atleast_1d(R_val).reshape(-1, 1) / Units.feet
    theta = np.atleast_1d(theta_raw).reshape(-1, 1)
    freq = np.atleast_1d(frequency).reshape(1, -1)
    
    M = segment.state.conditions.freestream.mach_number
    rho_0 = segment.state.conditions.freestream.density / Units["slugs/ft^3"]
    c_0 = segment.state.conditions.freestream.speed_of_sound / Units["ft/s"]
    velocity = segment.state.conditions.freestream.velocity / Units["ft/s"]
    Ls = Ls / Units.feet

    U_eff = velocity * np.cos(sigma_s)
    M_eff = M * np.cos(sigma_s)
    
    St = (freq * Ls) / U_eff
    St_peak = 2.0 
    F_St = (St / St_peak)**2 / ((1 + (St / St_peak)**2)**3.5)
    
    local_accel_factor = 1.0 + 2.0 * np.sin(alpha)
    M_local = M_eff * local_accel_factor
    W_M = M_local**4.5
    
    # Vectorized Doppler Factor (Using np.where to prevent 2D broadcast collapse)
    doppler_factor = np.ones_like(theta)
    doppler_factor = np.where(theta < (np.pi/2), 1.0 - M * np.cos(theta), doppler_factor)
    doppler_factor = np.where(theta > (np.pi/2), 1.0 + M * np.cos(theta), doppler_factor)
    convective_amplification = doppler_factor**(-2)
    
    total_pitch = alpha + gamma_s
    cos_theta_local = (np.cos(theta) * np.cos(total_pitch) + np.sin(theta) * np.sin(phi) * np.sin(total_pitch))
    D_theta_phi = (1.0 - cos_theta_local**2) * np.cos(sigma_s)**2
    
    ambient_scale = (rho_0 * c_0**2)**2
    spherical_spreading = 1 / (distance**2)
    
    # Broadcast computation (N_mic, N_freq)
    Pi = A * ambient_scale * W_M * spherical_spreading * convective_amplification * D_theta_phi * F_St
    
    p_ref_psf = (2*10**-5)/Units.psf 
    # Convert to 1/3 Octave Band Sound Pressure Level (dB)
    SPL = 10.0 * np.log10(Pi / (p_ref_psf**2) + 1e-12) + 10.0 * np.log10(0.231 * freq)
    return SPL

# ----------------------------------------------------------------------------------------------------------------------
#  Landing Gear Noise Model 
# ----------------------------------------------------------------------------------------------------------------------
def compute_landing_gear_noise(R_val, theta_raw, D, H, W, wheels, M, Weight, strut_diameter, frequency, segment):
    gear_params = Data(
        num_wheels = wheels,
        wheel_diam = D / Units.inches,                  
        wheel_width = W / Units.inches,                 
        strut_lengths = [H / Units.inches],             
        strut_dims = [strut_diameter / Units.inches],   
        aircraft_weight = Weight / Units.pounds,        
        track_angle = 0.0,                              
    )

    flight_cond = Data(
        M_flight = M / 0.75, 
        theta = np.degrees(np.atleast_1d(theta_raw).reshape(-1, 1)), 
        R = np.atleast_1d(R_val).reshape(-1, 1) / Units.feet,
        c0 = (segment.state.conditions.freestream.speed_of_sound / Units.foot_per_second), 
        rho0 = segment.state.conditions.freestream.density / Units["slugs/ft^3"]   
    )

    return predict_spectrum(np.atleast_1d(frequency).reshape(1, -1), gear_params, flight_cond)

P_REF_VAL = (2*10**-5)/Units.psf 
NOISE_PARAMS = Data(
    Low = Data(beta= 4.5e-8, St0= 1.0, sigma= 4.0, mu= 2.5, q= 2.6, h= 0.2, A= 3.53, B= 0.62),
    Mid = Data(beta= 1.5e-8, St0= 0.3, sigma= 3.0, mu= 1.5, q= 4.2, h= 0.6, A= 0.42, B= 0.18),
    High = Data(beta= 3.2e-5, St0= 0.1, sigma= 2.0, mu= 1.1, q= 4.2, h= 1.0, A= 0.08, B= 0.10)
)

def calculate_geometry(gear_params):
    Nw, w_ft, d_ft = gear_params.num_wheels, gear_params.wheel_width/12, gear_params.wheel_diam/12
    L_struts_ft, D_struts_ft = np.array(gear_params.strut_lengths)/12, np.array(gear_params.strut_dims)/12
    W_ac, gamma = gear_params.aircraft_weight, np.radians(gear_params.track_angle)
    
    S_L, l0_L = np.pi * Nw * w_ft * d_ft, d_ft
    perimeters = np.pi * D_struts_ft
    S_M = np.sum(perimeters * L_struts_ft)
    L_total_ft = np.sum(L_struts_ft)
    L_total_in = L_total_ft * 12.0
    
    a_ft = (S_M / (np.pi * L_total_ft)) if L_total_ft > 0 else 1.0
    l0_M = a_ft
    
    N_ref, W_ref, L_ref = 2.0, 150000.0, 300.0
    term1 = 1 + 0.028 * ((Nw / N_ref) * (L_total_in / L_ref) * (W_ac / W_ref) - 1)
    term2 = 1 + 2 * ((Nw - 2) / Nw) * np.sin(2 * gamma) if Nw > 2 else 1.0
        
    eta = term1 * term2
    l0_H = 0.15 * l0_M
    S_H = eta * (l0_H**2)
    
    return Data(Low=Data(S=S_L, l0=l0_L), Mid=Data(S=S_M, l0=l0_M), High=Data(S=S_H, l0=l0_H))

def normalized_spectrum(St, comp_type):
    p = NOISE_PARAMS[str(comp_type)]
    return (p.A * (St**p.sigma)) / ((p.B + St**p.mu)**p.q)

def directivity_component(theta_deg, comp_type):
    h = NOISE_PARAMS[str(comp_type)].h
    return (1 + h * np.cos(np.radians(theta_deg))**2)**2

def to_db(p2):
    return 10 * np.log10(np.maximum(p2, 1e-20) / (P_REF_VAL**2))

def predict_spectrum(freq, gear_params, flight_params):
    M_local = 0.75 * flight_params.M_flight
    geom = calculate_geometry(gear_params)
    
    amb_term = (flight_params.rho0 * flight_params.c0**2)**2
    P_base = (amb_term * M_local**6) / (flight_params.R**2)
    U = M_local * flight_params.c0
    
    # Vectorized computations across Frequencies & Receptors
    St_L = freq * geom['Low']['l0'] / U
    val_L = P_base * NOISE_PARAMS.Low.beta * geom.Low.S * directivity_component(flight_params.theta, 'Low') * normalized_spectrum(St_L, 'Low')
    
    St_M = freq * geom['Mid']['l0'] / U
    val_M = P_base * NOISE_PARAMS.Mid.beta * geom.Mid.S * directivity_component(flight_params.theta, 'Mid') * normalized_spectrum(St_M, 'Mid')
    
    St_H = freq * geom['High']['l0'] / U
    val_H = P_base * NOISE_PARAMS.High.beta * geom.High.S * directivity_component(flight_params.theta, 'High') * normalized_spectrum(St_H, 'High')
    
    val_total = val_L + val_M + val_H
    
    # Return 2D Data Array formats
    return Data(Freq=freq.flatten(), Total=to_db(val_total), Low=to_db(val_L), Mid=to_db(val_M), High=to_db(val_H))

# ----------------------------------------------------------------------------------------------------------------------     
#  Turbofan Fan Noise 
# ----------------------------------------------------------------------------------------------------------------------  
interpolator_fan = create_interpolator_fan()

def compute_fan_noise(R_val, theta_engine, turbofan, m, aeroacoustic_data, segment, frequencies):
    distance_microphone = np.atleast_1d(R_val).reshape(-1, 1)
    theta_s = np.degrees(np.atleast_1d(theta_engine).reshape(-1, 1))
    freq = np.atleast_1d(frequencies).reshape(1, -1)
    
    N1 = aeroacoustic_data.propulsors[turbofan.tag].fan.angular_velocity
    Velocity_secondary = aeroacoustic_data.propulsors[turbofan.tag].fan.exit_velocity   
    Temperature_secondary = aeroacoustic_data.propulsors[turbofan.tag].fan.exit_stagnation_temperature 
    Temperature_static_output = aeroacoustic_data.propulsors[turbofan.tag].fan.static_temperature_output
    Temperature_static_input = aeroacoustic_data.propulsors[turbofan.tag].fan.static_temperature_input
    Pressure_secondary = aeroacoustic_data.propulsors[turbofan.tag].fan.exit_stagnation_pressure 
    Velocity_aircraft = segment.state.conditions.freestream.velocity
    Diameter_secondary = aeroacoustic_data.propulsors[turbofan.tag].fan.diameter
    Num_blades = aeroacoustic_data.propulsors[turbofan.tag].fan.number_of_blades

    sound_ambient = segment.state.conditions.freestream.speed_of_sound
    R_gas, gamma = 287.1, 1.4 
    Cp = R_gas / (1 - 1/gamma)
    
    density_secondary = Pressure_secondary / (R_gas * (Temperature_secondary - (0.5 * Velocity_secondary**2 / Cp)))
    delt_T = Temperature_static_output - Temperature_static_input    
    M_TR = ((Velocity_aircraft**2 + ((np.pi*Diameter_secondary*N1)/60)**2)**0.5) / sound_ambient 

    if m is None:
            # CORRECTED: Calculate the annular area by subtracting the core nozzle area
            Diameter_primary = turbofan.core_nozzle.diameter
            Area_secondary = (np.pi / 4.0) * (Diameter_secondary**2 - Diameter_primary**2)
            
            # Removed the arbitrary `m - 600` hack
            m = (Area_secondary * Velocity_secondary * density_secondary) / Units.lbs

    fan_inputs = Data(
        m = m, 
        delta_T = delt_T * 1.8,                                                  
        M_TR = M_TR,                                                                   
        RSS = 150.0,                                                                   
        f_b = (N1 * Num_blades * Diameter_secondary) / Units.minute,                                          
        M_Tip = (np.pi * Diameter_secondary * N1 * (1/60)) / (sound_ambient),                   
        V_Number = 54,                                                     
        B_Number = Num_blades,                                             
        inlet_distortion = False                                           
    )

    def calc_base_level(inputs):
        return 10 * math.log10(inputs.m / 1.0) + 40 * math.log10(inputs.delta_T / 1.0)

    def calc_combination_tones(inputs, f, theta):
        # Use safe wrapper to avoid crash if RCAIDE interpolator strictly requires scalars
        F3 = _safe_interp_1d(get_spl_fan, interpolator_fan, "Combination Tones", theta)
        F2 = 0
        M_TR, f_b = inputs.M_TR, inputs.f_b
        base_spl = calc_base_level(inputs)
        
        F1_half = -18 + 46.5*(M_TR - 1)/0.146 if M_TR < 1.146 else 28.5 - 12*(M_TR - 1.146)/0.854
        F4_half = np.where(f < 0.5 * f_b, 20 * np.log10(f / (0.5 * f_b)), -20 * np.log10(f / (0.5 * f_b)))
        spl_half = base_spl + F1_half + F2 + F3 + F4_half

        F1_quarter = -15 + 47.5*(M_TR - 1)/0.322 if M_TR < 1.322 else 32.5 - 9*(M_TR - 1.322)/0.678
        F4_quarter = np.where(f < 0.25 * f_b, 30 * np.log10(f / (0.25 * f_b)), -30 * np.log10(f / (0.25 * f_b)))
        spl_quarter = base_spl + F1_quarter + F2 + F3 + F4_quarter

        F1_eighth = -12 + 41.2*(M_TR - 1)/0.61 if M_TR < 1.61 else 29.2 - 4.7*(M_TR - 1.61)/0.39
        F4_eighth = np.where(f < 0.125 * f_b, 30 * np.log10(f / (0.125 * f_b)), -20 * np.log10(f / (0.125 * f_b)))
        spl_eighth = base_spl + F1_eighth + F2 + F3 + F4_eighth

        return 10 * np.log10(10**(spl_half/10) + 10**(spl_quarter/10) + 10**(spl_eighth/10))
    
    spl = calc_combination_tones(fan_inputs, freq, theta_s)
    distance_attenuated_spl = spl + 20 * np.log10(0.25 / distance_microphone) 
    
    # Pack into (1, N_mic, N_freq) expected shapes for segment output
    fan_noise = Data()
    fan_noise.SPL_1_3_spectrum = np.expand_dims(distance_attenuated_spl, axis=0) 
    fan_noise.SPL = np.expand_dims(SPL_arithmetic(distance_attenuated_spl, sum_axis=1), axis=0)
    fan_noise.SPL_dBA = np.expand_dims(SPL_arithmetic(np.atleast_2d(A_weighting_metric(distance_attenuated_spl, freq.flatten())), sum_axis=1), axis=0)
    
    return fan_noise

# ----------------------------------------------------------------------------------------------------------------------     
#  Turbofan Engine Core Noise
# ---------------------------------------------------------------------------------------------------------------------- 
interp_C1 = create_interpolator("table_c1")
interp_C2 = create_interpolator("table_c2")
interp_C3 = create_interpolator("table_c3")

def compute_core_noise(R_val, theta_engine, turbofan, pr, aeroacoustic_data, segment, frequencies):
    
    distance_microphone = np.atleast_1d(R_val).reshape(-1, 1) / Units.feet
    theta_c = np.degrees(np.atleast_1d(theta_engine).reshape(-1, 1))
    freq = np.atleast_1d(frequencies).reshape(1, -1)
    
    Velocity_primary = aeroacoustic_data.propulsors[turbofan.tag].core_nozzle.exit_velocity  
    Temperature_primary = aeroacoustic_data.propulsors[turbofan.tag].core_nozzle.exit_stagnation_temperature 
    Pressure_primary = aeroacoustic_data.propulsors[turbofan.tag].core_nozzle.exit_stagnation_pressure      
    Diameter_primary = turbofan.core_nozzle.diameter
    Num_nozzle = turbofan.combustor.number_of_fuel_nozzle
    
    sound_ambient = segment.state.conditions.freestream.speed_of_sound
    pressure_amb = segment.state.conditions.freestream.pressure 
    temp_amb = segment.state.conditions.freestream.temperature
    R_gas, gamma_primary = 287.1, 1.37  
    Cpp = R_gas / (1 - 1/gamma_primary)
    density_primary = Pressure_primary / (R_gas * Temperature_primary - (0.5 * R_gas * Velocity_primary**2 / Cpp)) 

    model_inputs = Data(
        W1 = ((np.pi * (Diameter_primary/2)**2 * Velocity_primary * density_primary) / Units.lbm),  
        T_C_o = segment.state.conditions.energy.converters['combustor'].outputs.static_temperature * 1.8,             
        T_C_i = segment.state.conditions.energy.converters['combustor'].inputs.static_temperature * 1.8,              
        P_amb = pressure_amb / Units.psi,                                               
        T_amb = (temp_amb * 1.8)[0][0],                                                           
        n_f = Num_nozzle,                                                              
        R = distance_microphone, 
        D_h_1 = turbofan.core_nozzle.diameter / Units.feet,                                      
        c_amb = (sound_ambient / Units.feet)[0][0],                                             
        D_C = turbofan.combustor.diameter / Units.feet,                                          
        c_C_o = (331.3 * (1 + ((segment.state.conditions.energy.converters['combustor'].outputs.static_temperature-273)/273))**0.5) / Units.feet,      
        f = freq,                                                                                     
        theta_c = theta_c,     
        pressure_ratio = pr  
    )

    core_param_log = calc_core_param(model_inputs.W1, model_inputs.T_C_o, model_inputs.T_C_i, model_inputs.pressure_ratio, model_inputs.T_amb)

    uol_c1 = calc_uol_c1(model_inputs.R, model_inputs.n_f, core_param_log)
    uol_c2 = calc_uol_c2(model_inputs.R, model_inputs.n_f, core_param_log)
    uol_c3 = calc_uol_c3(model_inputs.R, core_param_log)
    
    s_c1 = calc_strouhal_c1(model_inputs.f, model_inputs.D_h_1, model_inputs.c_amb)
    s_c2_c3 = calc_strouhal_c2_c3(model_inputs.f, model_inputs.D_C, model_inputs.c_C_o) 
    
    norm_spl_c1 = get_normalized_spl(interp_C1, s_c1, model_inputs.theta_c) 
    norm_spl_c2 = get_normalized_spl(interp_C2, s_c2_c3, model_inputs.theta_c)
    norm_spl_c3 = get_normalized_spl(interp_C3, s_c2_c3, model_inputs.theta_c)
    
    c1_arr = norm_spl_c1 + uol_c1
    c2_arr = norm_spl_c2 + uol_c2
    c3_arr = norm_spl_c3 + uol_c3
    
    SPL_total = 10 * np.log10(10**(c1_arr/10) + 10**(c2_arr/10) + 10**(c3_arr/10))
    
    core_noise = Data()   
    core_noise.SPL_1_3_spectrum = np.expand_dims(SPL_total, axis=0)
    core_noise.SPL = np.expand_dims(SPL_arithmetic(SPL_total, sum_axis=1), axis=0)
    core_noise.SPL_dBA = np.expand_dims(SPL_arithmetic(np.atleast_2d(A_weighting_metric(SPL_total, freq.flatten())), sum_axis=1), axis=0)

    return core_noise 

def get_normalized_spl(interpolator_obj, strouhal_num, theta_c):
    # CORRECTED: Clip Strouhal numbers to prevent wild extrapolations outside empirical bounds
    clamped_strouhal = np.clip(strouhal_num, 1e-2, 1e2)
    st_bcast = np.broadcast_to(clamped_strouhal, (theta_c.shape[0], clamped_strouhal.shape[1]))
    
    mask = st_bcast > 0
    log_S = np.full_like(st_bcast, -10.0, dtype=float)
    log_S[mask] = np.log10(st_bcast[mask])
    
    Theta = np.broadcast_to(theta_c, st_bcast.shape)
    
    try:
        res = interpolator_obj.ev(log_S.ravel(), Theta.ravel()).reshape(st_bcast.shape)
    except AttributeError:
        res = interpolator_obj(np.column_stack((log_S.ravel(), Theta.ravel()))).reshape(st_bcast.shape)
        
    res[~mask] = 0.0
    return res


def calc_core_param(W1, T_C_o, T_C_i, pressure_ratio, T_amb):
    return np.log10(W1 * (((T_C_o - T_C_i) * pressure_ratio * (T_amb / T_C_i)) ** 2))

def calc_uol_c1(R, n_f, core_param_log):
    return 78.0 - (20.0 * np.log10(R)) + (7.0 * core_param_log) - (14.0 * np.log10(n_f))

def calc_uol_c2(R, n_f, core_param_log):
    return 60.3 - (20.0 * np.log10(R)) + (10.0 * core_param_log) - (18.0 * np.log10(n_f))

def calc_uol_c3(R, core_param_log):
    return 42.5 - (20.0 * np.log10(R)) + (9.0 * core_param_log)

def calc_strouhal_c1(f, D_h_1, c_amb):
    return (f * D_h_1) / c_amb

def calc_strouhal_c2_c3(f, D_C, c_C_o):
    return (f * D_C) / c_C_o