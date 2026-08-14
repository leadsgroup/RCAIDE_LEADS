# RCAIDE/Library/Methods/Aeroacoustics/Semi_Empirical/Airframe/landing_gear_noise_model.py
#
# Created:  Jun 2026, M. Clarke, P. Siripun

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
def compute_landing_gear_noise(R_val, theta_raw, landing_gear, vehicle, cpt, frequency, segment, aeroacoustics_analysis):
    """
        This calculates the Landing gear overall sound pressure level.

        Parameters
        ----------
        R_val : array_like
            Distance from aircraft to each receptor [m]
        theta_raw : array_like
            Emission angle to each receptor [rad] (pi/2 is overhead)
        landing_gear : RCAIDE.Library.Components.Landing_Gear.Landing_Gear
            Provides tire_diameter, strut_length, tire_width, wheels, strut_diameter, and
            (for Main_Landing_Gear) units
        vehicle : RCAIDE.Vehicle
            Provides vehicle.mass_properties.max_takeoff
        cpt : int or array_like
            Control point index (or one index per receptor, matching R_val/theta_raw when
            batching multiple control points in one call) into segment.state.conditions
        aeroacoustics_analysis : RCAIDE.Framework.Analyses.Aeroacoustics.Semi_Empirical
            Provides the empirical Low/Mid/High band correlation coefficients via
            aeroacoustics_analysis.settings.landing_gear_noise_parameters

    Returns
    -------
    SPL : array_like of Low, Mid and High Frequency Spectrum
        Sound Pressure Level of the landing gear [dB].

    Notes
    -----
    The function uses correlation-based methods to compute the noise levels from the landing gear.
    Inputs/outputs are vectorized across receptors (R_val, theta_raw) and frequency to support
    evaluating multiple ground receptors in one call. Identical gear units (e.g. two main gear
    legs sharing one Main_Landing_Gear component) are summed incoherently.

    **Definitions**

    'SPL'
        Sound Pressure Level, a measure of the sound intensity.

    References
    ----------
    Guo, Yueping. "A Semi-Empirical Model for Aircraft Landing Gear Noise Prediction." AIAA 2006-2627.
    """

    noise_params = aeroacoustics_analysis.settings.landing_gear_noise_parameters
    freq = np.atleast_1d(frequency).reshape(1, -1)

    # --- geometry (Guo AIAA 2006-2627) ---
    Nw         = landing_gear.wheels
    w_ft       = landing_gear.tire_width / Units.inches / 12
    d_ft       = landing_gear.tire_diameter / Units.inches / 12
    L_strut_in = landing_gear.strut_length / Units.inches
    L_strut_ft = L_strut_in / 12
    D_strut_ft = landing_gear.strut_diameter / Units.inches / 12
    W_ac       = vehicle.mass_properties.max_takeoff / Units.pounds

    S_L, l0_L = np.pi * Nw * w_ft * d_ft, d_ft
    S_M       = np.pi * D_strut_ft * L_strut_ft
    l0_M      = (S_M / (np.pi * L_strut_ft)) if L_strut_ft > 0 else 1.0

    N_ref, W_ref, L_ref = 2.0, 150000.0, 300.0
    complexity = 1 + 0.028 * ((Nw / N_ref) * (L_strut_in / L_ref) * (W_ac / W_ref) - 1)
    l0_H = 0.15 * l0_M
    S_H  = complexity * (l0_H**2)   # track angle assumed zero, so the wheel-track term is 1

    # --- flight condition ---
    # cpt may be a scalar or an array of per-receptor control-point indices; indexing with
    # np.atleast_1d(cpt) (rather than [cpt, 0]) keeps the trailing column dimension so these
    # broadcast correctly against the (n_receptor, 1) R_val/theta_raw arrays either way.
    cpt_idx   = np.atleast_1d(cpt)
    M         = segment.state.conditions.freestream.mach_number[cpt_idx]
    M_local   = 0.75 * M   # Local Mach number at the gear (Eq. 58)
    c0        = segment.state.conditions.freestream.speed_of_sound[cpt_idx] / Units.foot_per_second
    rho0      = segment.state.conditions.freestream.density[cpt_idx] / Units["slugs/ft^3"]
    theta_deg = np.degrees(np.atleast_1d(theta_raw).reshape(-1, 1))
    R_ft      = np.atleast_1d(R_val).reshape(-1, 1) / Units.feet
    U         = M_local * c0

    amb_term = (rho0 * c0**2)**2
    P_base   = (amb_term * M_local**6) / (R_ft**2)

    St_L  = freq * l0_L / U
    val_L = P_base * noise_params.Low.beta * S_L * directivity_component(theta_deg, 'Low', noise_params) * normalized_spectrum(St_L, 'Low', noise_params)

    St_M  = freq * l0_M / U
    val_M = P_base * noise_params.Mid.beta * S_M * directivity_component(theta_deg, 'Mid', noise_params) * normalized_spectrum(St_M, 'Mid', noise_params)

    St_H  = freq * l0_H / U
    val_H = P_base * noise_params.High.beta * S_H * directivity_component(theta_deg, 'High', noise_params) * normalized_spectrum(St_H, 'High', noise_params)

    units = getattr(landing_gear, 'units', 1)
    if units > 1:
        # Incoherent summation of identical gear units (e.g. left/right main gear legs):
        # scaling mean-square pressure by `units` is equivalent to a +10*log10(units) dB adjustment
        val_L, val_M, val_H = val_L * units, val_M * units, val_H * units

    val_total = val_L + val_M + val_H

    return Data(Freq=freq.flatten(), Total=to_db(val_total), Low=to_db(val_L), Mid=to_db(val_M), High=to_db(val_H))


# Reference pressure defining the dB SPL scale (20 micropascal), not a tunable model parameter
P_REF_VAL = (2*10**-5)/Units.psf  # [psf]

def normalized_spectrum(St, comp_type, noise_params):
    """Calculates F(St) using Eq. 43."""
    p = noise_params[str(comp_type)]
    return (p.A * (St**p.sigma)) / ((p.B + St**p.mu)**p.q)

def directivity_component(theta_deg, comp_type, noise_params):
    """Calculates D(theta) using Eq. 51."""
    h = noise_params[str(comp_type)].h
    return (1 + h * np.cos(np.radians(theta_deg))**2)**2

def to_db(p2):
    """Converts mean squared acoustic pressure to Sound Pressure Level (dB)."""
    return 10 * np.log10(np.maximum(p2, 1e-20) / (P_REF_VAL**2))
