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
#  Landing Gear Noise Model 
# ----------------------------------------------------------------------------------------------------------------------

def compute_landing_gear_noise(R_val, theta_raw, D, H, W, wheels, M, Weight, strut_diameter, frequency, segment):
    """
        This calculates the Landing gear overall sound pressure level.

        Parameters
        
        gear_params: dict
            - num_wheels (Nw): Number of wheels
            - wheel_diam (d): Wheel diameter [inches]
            - wheel_width (w): Wheel width [inches]
            - strut_lengths (L_j): List of lengths of struts [inches]
            - strut_dims (dim_j): List of diameters/widths of struts [inches]
            - aircraft_weight (W_ac): Max Takeoff Weight [lbs]
            - track_angle (gamma): Wheel track alignment angle [degrees]
            
        flight_params: dict
            - M_flight: Flight Mach number
            - theta: Emission angle [degrees] (90 is overhead)
            - R: Distance to observer [ft]
            - c0: Speed of sound [ft/s] (default 1116)
            - rho0: Air density [slugs/ft^3] (default 0.00237)

    Returns
    -------
    SPL : array_like of Low, Mid and High Frequency Spectrum
        Sound Pressure Level of the landing gear [dB].

    Notes
    -----
    The function uses correlation-based methods to compute the noise levels from the landing gear.

    **Definitions**

    'SPL'
        Sound Pressure Level, a measure of the sound intensity.

    References
    ----------
    Guo, Yueping. "A Semi-Empirical Model for Aircraft Landing Gear Noise Prediction." AIAA 2006-2627.  
    """

    gear_params = Data(
        num_wheels = wheels,
        wheel_diam = D / Units.inches,                  # Approximate (in)
        wheel_width = W / Units.inches,                 # Approximate (in)
        strut_lengths = [H / Units.inches],             # Total length L=317 in
        strut_dims = [strut_diameter / Units.inches],   # Average dimension a=4.65 in
        aircraft_weight = Weight / Units.pounds,        # Reference weight (lbs)
        track_angle = 0.0,                              # Assume track angle of zero
    )


    flight_cond = Data(
        M_flight = M / 0.75, 
        theta = theta_raw[0][0]/ Units.deg, # (deg),
        R = R_val[0][0]/Units.feet, #convert back to feet from m
        c0 = (segment.state.conditions.freestream.speed_of_sound / Units.foot_per_second), # sound speed (ft/s)
        rho0 = segment.state.conditions.freestream.density / Units["slugs/ft^3"]   # slug/ft^3
    )

    return predict_spectrum(frequency, gear_params, flight_cond)


# --- Global Constants & Empirical Parameters ---
P_REF_VAL = (2*10**-5)/Units.psf  # Reference pressure [psf]

NOISE_PARAMS = Data(
    Low = Data(
        beta= 4.5e-8, St0= 1.0, sigma= 4.0, mu= 2.5, q= 2.6, 
        h= 0.2, A= 3.53, B= 0.62
    ),
    Mid = Data(
        beta= 1.5e-8, St0= 0.3, sigma= 3.0, mu= 1.5, q= 4.2, 
        h= 0.6, A= 0.42, B= 0.18
    ),
    High = Data(
        beta= 3.2e-5, St0= 0.1, sigma= 2.0, mu= 1.1, q= 4.2, 
        h= 1.0, A= 0.08, B= 0.10
    )
)

# --- Core Functions ---

def calculate_geometry(gear_params):
    """Calculates S (Area), l0 (Length scale), and complexity factor for each regime."""
    Nw = gear_params.num_wheels 
    w_ft = gear_params.wheel_width /12
    d_ft = gear_params.wheel_diam /12
    L_struts_ft = np.array(gear_params.strut_lengths) /12
    D_struts_ft = np.array(gear_params.strut_dims) /12
    W_ac = gear_params.aircraft_weight
    gamma = np.radians(gear_params.track_angle)
    
    # --- LOW FREQ (Wheels) ---
    S_L = np.pi * Nw * w_ft * d_ft
    l0_L = d_ft
    
    # --- MID FREQ (Struts) ---
    perimeters = np.pi * D_struts_ft
    S_M = np.sum(perimeters * L_struts_ft)
    
    L_total_ft = np.sum(L_struts_ft)
    L_total_in = L_total_ft * 12.0
    
    # Average dimension cross-section
    a_ft = (S_M / (np.pi * L_total_ft)) if L_total_ft > 0 else 1.0
    l0_M = a_ft
    
    # --- HIGH FREQ (Complexity) ---
    N_ref, W_ref, L_ref = 2.0, 150000.0, 300.0
    
    term1 = 1 + 0.028 * ((Nw / N_ref) * (L_total_in / L_ref) * (W_ac / W_ref) - 1)
    
    # Guard for cases where Nw <= 2
    term2 = 1.0
    if Nw > 2:
        term2 = 1 + 2 * ((Nw - 2) / Nw) * np.sin(2 * gamma)
        
    eta = term1 * term2
    l0_H = 0.15 * l0_M
    S_H = eta * (l0_H**2)
    
    return Data(
        Low = Data(S= S_L, l0= l0_L),
        Mid = Data(S= S_M, l0= l0_M),
        High= Data(S= S_H, l0= l0_H)
    )

def normalized_spectrum(St, comp_type):
    """Calculates F(St) using Eq. 43."""
    p = NOISE_PARAMS[str(comp_type)]
    num = p.A * (St**p.sigma)
    den = (p.B + St**p.mu)**p.q
    return num / den

def directivity_component(theta_deg, comp_type):
    """Calculates D(theta) using Eq. 51."""
    h = NOISE_PARAMS[str(comp_type)].h
    theta_rad = np.radians(theta_deg)
    return (1 + h * np.cos(theta_rad)**2)**2

def to_db(p2):
    """Converts mean squared acoustic pressure to Sound Pressure Level (dB)."""
    p2 = np.maximum(p2, 1e-20)
    return 10 * np.log10(p2 / (P_REF_VAL**2))

def predict_spectrum(frequencies, gear_params, flight_params):
    """Calculates SPL for a list of frequencies."""
    c0 = flight_params.c0
    rho0 = flight_params.rho0
    M_flight = flight_params.M_flight
    theta_deg = flight_params.theta
    R_ft = flight_params.R
    
    # Local Mach number (Eq. 58)
    M_local = 0.75 * M_flight
    
    # Retrieve geometries
    geom = calculate_geometry(gear_params)
    
    # Base Amplitude Term (Eq 31)
    amb_term = (rho0 * c0**2)**2
    conv_term = 1.0  # Convective amplification removed per original code
    spread_term = R_ft**2
    
    P_base = (amb_term * M_local**6) / (spread_term * conv_term)
    D0 = 1.0  # Installation effect
    U = M_local * c0
    Low = []
    Mid = []
    High = []
    Total = []
    for f in frequencies:
        # --- Low ---
        St_L = f * geom['Low']['l0'] / U
        val_L = P_base * D0 * NOISE_PARAMS.Low.beta * geom.Low.S * \
                directivity_component(theta_deg, 'Low') * \
                normalized_spectrum(St_L, 'Low')
        
        # --- Mid ---
        St_M = f * geom['Mid']['l0'] / U
        val_M = P_base * D0 * NOISE_PARAMS.Mid.beta * geom.Mid.S * \
                directivity_component(theta_deg, 'Mid') * \
                normalized_spectrum(St_M, 'Mid')
        
        # --- High ---
        St_H = f * geom['High']['l0'] / U
        val_H = P_base * D0 * NOISE_PARAMS.High.beta * geom.High.S * \
                directivity_component(theta_deg, 'High') * \
                normalized_spectrum(St_H, 'High')
        
        val_total = val_L + val_M + val_H
        
        # Append dB results
        Low.append(to_db(val_L)[0][0])
        Mid.append(to_db(val_M)[0][0])
        High.append(to_db(val_H)[0][0])
        Total.append(to_db(val_total)[0][0])
    results = Data(Freq= frequencies, Total= Total, Low= Low, Mid= Mid, High= High)
    return results

