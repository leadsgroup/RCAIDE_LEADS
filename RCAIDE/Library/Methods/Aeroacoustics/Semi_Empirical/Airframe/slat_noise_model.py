# RCAIDE/Library/Methods/Aeroacoustics/Semi_Empirical/Airframe/slat_noise_model.py
#
# Created:  Jun 2026, M. Clarke, P. Siripun

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Framework.Core import Units

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Slat Noise Model
# ----------------------------------------------------------------------------------------------------------------------
def slat_noise(R_val, theta_raw, control_surface, wing, cpt, segment, frequency, aeroacoustics_analysis, phi=0.0):
    """
    Computes the Slat Noise Power Spectral Density based on Guo (2010).

    Inputs are vectorized across receptors (R_val, theta_raw) and frequency: R_val/theta_raw
    broadcast as column vectors (n_receptor, 1), frequency as a row vector (1, n_freq), giving an
    (n_receptor, n_freq) SPL array.

    Parameters
    -----------
    R_val : float
        Distance from airplane to observer (r) [m].
    theta_raw : float
        Polar angle [rad].
    control_surface : RCAIDE.Library.Components.Wings.Control_Surfaces.Slat
        Provides span_fraction_start/end, chord_fraction, deflection
    wing : RCAIDE.Library.Components.Wings.Wing
        Parent wing, provides taper/spans/areas/sweeps
    cpt : int or array_like
        Control point index (or one index per receptor, matching R_val/theta_raw when
        batching multiple control points in one call) into segment.state.conditions
    frequency : array_like
        Frequency array (f) [Hz].
    aeroacoustics_analysis : RCAIDE.Framework.Analyses.Aeroacoustics.Semi_Empirical
        Provides the empirical amplitude calibration constant and spectral peak Strouhal
        number via aeroacoustics_analysis.settings.slat_noise_parameters
    phi : float
        Azimuthal angle [rad], defaults to 0 (zero bank angle, matching the level-flight
        assumption used throughout this model)

    Returns:
    --------
    SPL : array_like
        Sound Pressure Level spectrum [dB] at the given frequencies.
    """
    # Average chord of the slat across its span, from the wing's taper/span/area and the
    # slat's own span/chord fractions
    taper       = wing.taper
    root_chord  = 2 * wing.areas.reference / wing.spans.projected / (1 + taper)
    tip_chord   = taper * root_chord
    chord_start = root_chord + (tip_chord - root_chord) * control_surface.span_fraction_start
    chord_end   = root_chord + (tip_chord - root_chord) * control_surface.span_fraction_end
    Ls          = 0.5 * (chord_start + chord_end) * control_surface.chord_fraction

    gamma_s = control_surface.deflection
    sigma_s = wing.sweeps.leading_edge if wing.sweeps.leading_edge is not None else \
              (wing.sweeps.quarter_chord if wing.sweeps.quarter_chord is not None else 0.0)

    # cpt may be a scalar or an array of per-receptor control-point indices; indexing with
    # np.atleast_1d(cpt) (rather than [cpt, 0]) keeps the trailing column dimension so these
    # broadcast correctly against the (n_receptor, 1) distance/theta arrays either way.
    cpt_idx = np.atleast_1d(cpt)
    alpha   = segment.state.conditions.aerodynamics.angles.alpha[cpt_idx]

    distance = np.atleast_1d(R_val).reshape(-1, 1) / Units.feet
    theta = np.atleast_1d(theta_raw).reshape(-1, 1)
    freq = np.atleast_1d(frequency).reshape(1, -1)

    slat_params = aeroacoustics_analysis.settings.slat_noise_parameters
    A = slat_params.amplitude
    St_peak = slat_params.St_peak

    M = segment.state.conditions.freestream.mach_number[cpt_idx]
    rho_0 = segment.state.conditions.freestream.density[cpt_idx] / Units["slugs/ft^3"]
    c_0 = segment.state.conditions.freestream.speed_of_sound[cpt_idx] / Units["ft/s"]
    velocity = segment.state.conditions.freestream.velocity[cpt_idx] / Units["ft/s"]
    Ls = Ls / Units.feet

    U_eff = velocity * np.cos(sigma_s)
    M_eff = M * np.cos(sigma_s)

    # Strouhal Number Calculation
    St = (freq * Ls) / U_eff

    # Spectral Shape Function F(St)
    F_St = (St / St_peak)**2 / ((1 + (St / St_peak)**2)**3.5)

    # Angle of Attack Correction for Local Mach Number
    local_accel_factor = 1.0 + 2.0 * np.sin(alpha)
    M_local = M_eff * local_accel_factor

    # Mach Number Dependence W(M) using the accelerated local flow
    W_M = M_local**4.5

    # Convective Amplification (Doppler Factor). The cosine naturally handles the forward/aft
    # sign flip, so no piecewise branching on theta is needed.
    doppler_factor = 1.0 - M * np.cos(theta)
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
    spherical_spreading = 1 / (distance**2)

    # Assemble Far-Field Noise Power Spectral Density (Pi)
    Pi = A * ambient_scale * W_M * spherical_spreading * convective_amplification * D_theta_phi * F_St

    # Convert to 1/3 octave band Sound Pressure Level (dB)
    p_ref_psf = (2*10**-5)/Units.psf  # reference lowest spl
    SPL = 10.0 * np.log10(Pi / (p_ref_psf**2) + 1e-12) + 10.0 * np.log10(0.231 * freq)

    return SPL
