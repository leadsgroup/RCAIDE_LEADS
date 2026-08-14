# RCAIDE/Library/Methods/Aeroacoustics/Semi_Empirical/Airframe/flap_noise_model.py
#
# Created:  Jun 2026, M. Clarke, P. Siripun

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Library.Methods.Geometry.Planform.compute_chord_length_from_span_location import compute_chord_length_from_span_location

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Flap Noise Model
# ----------------------------------------------------------------------------------------------------------------------
def flap_noise_model(R_val, theta_flap, control_surface, wing, cpt, frequency, segment, aeroacoustics_analysis):
    """
    Calculates the flap side-edge noise Sound Pressure Level spectrum.

    Inputs are vectorized across receptors (R_val, theta_flap) and frequency: R_val/theta_flap
    broadcast as column vectors (n_receptor, 1), frequency as a row vector (1, n_freq), giving an
    (n_receptor, n_freq) SPL array.

    Parameters
    ----------
    R_val : array_like
        Distance from aircraft to each receptor [m]
    theta_flap : array_like
        Polar angle to each receptor [rad] (overhead = 90 deg)
    control_surface : RCAIDE.Library.Components.Wings.Control_Surfaces.Flap
        Provides span_fraction_start/end, chord_fraction, deflection
    wing : RCAIDE.Library.Components.Wings.Wing
        Parent wing, provides chords.root/tip, spans.projected, thickness_to_chord
    cpt : int or array_like
        Control point index (or one index per receptor, matching R_val/theta_flap when
        batching multiple control points in one call) into segment.state.conditions
    aeroacoustics_analysis : RCAIDE.Framework.Analyses.Aeroacoustics.Semi_Empirical
        Provides the empirical model constants via aeroacoustics_analysis.settings.flap_noise_parameters

    References
    ----------
    Guo, Yueping. "Aircraft Flap Side Edge Noise Modeling and Prediction" (2012)
    """

    # Average chord of the flap across its span, from the wing's chord distribution (accounts
    # for segmented wings, since wing_planform derives chords.root/tip from the segment
    # breakdown) and the flap's own span/chord fractions
    half_span   = wing.spans.projected / 2
    chord_start = compute_chord_length_from_span_location(wing, control_surface.span_fraction_start * half_span)
    chord_end   = compute_chord_length_from_span_location(wing, control_surface.span_fraction_end * half_span)
    cf          = 0.5 * (chord_start + chord_end) * control_surface.chord_fraction

    thickness = wing.thickness_to_chord * cf
    deltaf    = control_surface.deflection

    # Vectorize inputs for broadcasting: R_val/theta as column vectors (N_mic, 1), freq as row (1, N_freq)
    r = np.atleast_1d(R_val).reshape(-1, 1)
    theta = np.atleast_1d(theta_flap).reshape(-1, 1)
    freq = np.atleast_1d(frequency).reshape(1, -1)

    constants = aeroacoustics_analysis.settings.flap_noise_parameters
    sigma_f = constants.sigma_f

    p_ref = 2e-5  # Reference SPL in Pascals (20 micropascal, defines the dB scale)
    # cpt may be a scalar or an array of per-receptor control-point indices; indexing with
    # np.atleast_1d(cpt) (rather than [cpt, 0]) keeps the trailing column dimension so these
    # broadcast correctly against the (n_receptor, 1) r/theta arrays either way.
    cpt_idx = np.atleast_1d(cpt)
    alpha = segment.state.conditions.aerodynamics.angles.alpha[cpt_idx]
    M     = segment.state.conditions.freestream.mach_number[cpt_idx]
    c0    = segment.state.conditions.freestream.speed_of_sound[cpt_idx]
    U     = segment.state.conditions.freestream.velocity[cpt_idx]
    rho0  = segment.state.conditions.freestream.density[cpt_idx]

    # Convective amplification / Doppler factor (Eq 4.4)
    Delta = 1.0 - M * np.cos(theta)

    # Calculate the Mach integral once for this specific Mach number
    I_M = calc_mach_integral(M, constants.mu0, constants.mu1, constants.mu2)

    # Initialize total PSD array with correct broadcasted shape (N_mic, N_freq)
    PSD_total = np.zeros(np.broadcast(theta, freq).shape, dtype=float)

    # Loop over the two distinct noise bands to get the two bumps
    for is_high_freq in [False, True]:
        # Switch characteristic lengths and power laws
        l = thickness if is_high_freq else cf
        n = 6 if is_high_freq else 5

        A_G = calc_geometric_amplitude(is_high_freq, constants.A0, sigma_f, gamma_f=deltaf, alpha=alpha, h=thickness, Lf=cf)
        A_F = 1.0  # Standard assumption if flow is bundled into A0
        W_M = (M**n) / I_M

        f_source = freq / Delta
        F_f = calc_spectral_shape(f_source, M, l, c0, U, constants.mu0, constants.mu1, constants.mu2)

        # Distance and absorption scaling
        length_scale = (cf * l) / ((Delta**2) * (r**2))
        atmospheric_absorption = np.exp(-constants.alpha_0 * r)

        # Combine all to get PSD
        PSD_component = (rho0**2) * (c0**4) * A_G * A_F * W_M * F_f * (l / c0) * length_scale * atmospheric_absorption
        PSD_total += PSD_component

    # Convert total PSD [Pa^2/Hz] to 1/3 octave band SPL [dB]
    SPL = 10.0 * np.log10(PSD_total / (p_ref**2) + 1e-12) + 10.0 * np.log10(0.231 * freq)
    return SPL


def calc_geometric_amplitude(is_high_freq, A0, sigma_f, gamma_f, alpha, h, Lf):
    """Calculates the geometric amplitude A_G (Equations 8.1 and 8.2)"""
    if not is_high_freq:
        # LF Calculate curve according to paper
        return A0 * (1.0 + np.sin(sigma_f)) * (np.sin(gamma_f)**2)
    else:
        # HF Calculate curve according to paper
        return A0 * (h / Lf) * (1.0 + np.sin(sigma_f)) * ((1.0 + np.sin(alpha))**2) * (np.sin(alpha + gamma_f)**4)

def calc_spectral_shape(f, M, l, c0, U, mu0, mu1, mu2):
    """Calculates the Spectral Shape Function F(f, M) (Equation 5.17)"""
    omega = 2.0 * np.pi * f  # angular frequency
    k0 = (omega * l) / c0
    St = (omega * l) / U

    term1 = 1.0 + (mu0**2) * (St**2)
    term2 = 1.0 + (mu1**2) * ((1.0 + M)**2) * (St**2)
    term3 = 1.0 + (mu2**2) * (k0**2)

    return (k0**2) / (term1 * term2 * term3)

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
    for n in range(3):
        arg = (sigma[n] * (k02 - k01)) / (1.0 + (sigma[n]**2) * k01 * k02)
        I_M += (Gamma[n] / sigma[n]) * np.arctan(arg)

    return I_M
