# RCAIDE/Library/Methods/Aeroacoustics/Semi_Empirical/Propulsion/Engine_Noise/compute_jet_noise.py
#
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Framework.Core                        import Data

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  turbofan engine noise
# ----------------------------------------------------------------------------------------------------------------------
def compute_jet_noise(microphone_locations, turbofan, cpt, segment, frequencies, flag):
    """
    Implements the Four-Source Model for Coaxial Jet Noise Prediction (Isothermal).
    Based on ISVR Technical Report No 326, Section 4.

    Inputs are vectorized across receptors: microphone_locations is (n_receptor, 3), cpt is
    either a single control point index (shared by every receptor) or one index per receptor
    (matching microphone_locations, when batching multiple control points in one call), and
    frequencies is (n_freq,).

    Parameters
    ----------
    turbofan : RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan
    cpt : int or array_like
        Control point index (or one index per receptor) into segment.state.conditions
    flag : int
        1 to apply the reduced-thrust (reverse/idle approach) 0.8x velocity scaling, else 0
    """
    # 1. Extract Geometry and Conditions
    Dp = turbofan.core_nozzle.diameter
    Ds = turbofan.fan_nozzle.diameter

    core_nozzle_out = segment.state.conditions.energy.converters[turbofan.core_nozzle.tag].outputs
    fan_nozzle_out  = segment.state.conditions.energy.converters[turbofan.fan_nozzle.tag].outputs

    # cpt may be a scalar or an array of per-receptor control-point indices; indexing with
    # np.atleast_1d(cpt) (rather than [cpt, 0]) keeps the trailing column dimension so these
    # broadcast correctly against the (n_receptor, n_freq) spectrum arrays below either way.
    cpt_idx = np.atleast_1d(cpt)

    # Isothermal conditions, one value per receptor
    Vp = core_nozzle_out.velocity[cpt_idx]
    Vs = fan_nozzle_out.velocity[cpt_idx]

    if flag == 1:
        Vp = Vp * 0.8
        Vs = Vs * 0.8
    freqs = np.atleast_1d(frequencies).reshape(1, -1)

    # Ambient properties (approximated for static standard day)
    c_0 = 343.0

    # 2. Aerodynamic Flow Model (Section 4.1.1)

    # Area calculations
    Ap = (np.pi / 4.0) * Dp**2
    As = (np.pi / 4.0) * (Ds**2 - Dp**2)

    # Velocity and Area Ratios
    lambda_val = Vs / Vp
    beta = As / Ap # Density ratio is 1 for isothermal

    # Effective Jet (Interaction Zone)
    # De = Dp * (1 + lambda^2 * beta)^(1/2)
    De = Dp * np.sqrt(1 + (lambda_val**2) * beta)

    # Mixed Jet (Fully Mixed)
    # Enforcing conservation of mass and momentum for isothermal flow:
    # Vm = Vp * (1 + lambda^2 * beta) / (1 + lambda * beta)
    Vm = Vp * (1 + (lambda_val**2) * beta) / (1 + lambda_val * beta)

    # Am = Ap * (1 + lambda * beta)^2 / (1 + lambda^2 * beta)
    Am = Ap * ((1 + lambda_val * beta)**2) / (1 + (lambda_val**2) * beta)
    Dm = np.sqrt(4.0 * Am / np.pi)

    # 3. Acoustic Model (Section 4.2.1)

    # Cut-off frequencies (where f * D / V = 1)
    f1_s = Vs / Ds
    f1_m = Vm / Dm

    # Line-of-sight distance and polar angle to each receptor (0 = Nose, 180 = Tail)
    mic_x, mic_y, mic_z = microphone_locations[:, 0:1], microphone_locations[:, 1:2], microphone_locations[:, 2:3]
    R_dist = np.maximum(np.sqrt(mic_x**2 + mic_y**2 + mic_z**2), 0.1)   # guard against log(0)
    theta_rad = np.arctan2(mic_y, mic_x)

    # --- Component A: Secondary Jet Shear Layer ---
    St_s = freqs * Ds / Vs
    base_spl_s = empirical_single_jet_spl(Vs, Ds, St_s, c_0, theta_rad)
    Fu = 1.0 / (1.0 + (f1_s / freqs)**2)
    SPL_s = base_spl_s + 10.0 * np.log10(Fu)

    # --- Component B: Effective Jet (Interaction Zone) ---
    St_e = freqs * De / Vp
    base_spl_e = empirical_single_jet_spl(Vp, De, St_e, c_0, theta_rad)
    SPL_e = base_spl_e - 7.0

    # --- Component C: Fully Mixed Jet ---
    St_m = freqs * Dm / Vm
    base_spl_m = empirical_single_jet_spl(Vm, Dm, St_m, c_0, theta_rad)
    Fd = 1.0 / (1.0 + (freqs / f1_m)**2)
    SPL_m = base_spl_m + 10.0 * np.log10(Fd)

    # --- Total Incoherent Sum ---
    SPL_sum = 10.0 * np.log10(10**(SPL_s/10.0) + 10**(SPL_e/10.0) + 10**(SPL_m/10.0))

    # Apply true spherical divergence (20 * log10(R))
    distance_correction = 20.0 * np.log10(R_dist)
    SPL_total = SPL_sum - distance_correction

    jet_noise = Data()
    jet_noise.SPL_1_3_spectrum = np.expand_dims(SPL_total, axis=0)
    return jet_noise

def empirical_single_jet_spl(V, D, St, c_0, theta_rad):
    """
    Tuned empirical spectral shape for a single jet with directivity.
    """
    St_peak = 0.65
    shape_factor = (St / St_peak)**1.2 / (1.0 + (St / St_peak)**2.0)
    OASPL = 140.0 + 80.0 * np.log10(V / c_0) + 20.0 * np.log10(D)
    
    # Empirical directivity index (DI) mimicking a jet noise plume
    peak_theta = np.radians(180.0)
    DI = 15.0 * np.exp(-((theta_rad - peak_theta)**2) / 0.15)
    
    # Normalize so 90 degrees evaluates to 0 dB to keep the previous calibration intact
    DI_90 = 15.0 * np.exp(-((np.radians(90.0) - peak_theta)**2) / 0.3)
    
    SPL = OASPL + 10.0 * np.log10(shape_factor) + (DI - DI_90)
    return SPL