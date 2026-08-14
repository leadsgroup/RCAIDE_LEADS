# RCAIDE/Library/Methods/Aeroacoustics/Common/compute_lateral_attenuation.py
#
# Created:  Aug 2026, P. Siripun

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Lateral Attenuation
# ----------------------------------------------------------------------------------------------------------------------
def compute_lateral_attenuation(l_seg_m, d_seg_m, bank_angle_deg=0.0):
    """
    Computes the AEDT lateral attenuation adjustment (LA_ADJ) for a wing-mounted civil
    aircraft according to SAE-AIR-5662.

    Parameters
    ----------
    l_seg_m : array_like
        Horizontal sideline distance from the flight track to the receptor [meters]
    d_seg_m : array_like
        Aircraft AGL altitude at the closest point of approach [meters]
    bank_angle_deg : float
        Aircraft bank angle [degrees], defaults to 0.0 for straight flight

    Returns
    -------
    LA_ADJ : array_like
        Total lateral attenuation adjustment [dB]
    beta_deg : array_like
        Elevation angle [degrees]

    References
    ----------
    SAE-AIR-5662: Method for Predicting Lateral Attenuation of Airplane Noise
    """
    # Geometry: slant range, guarded against divide-by-zero
    SLR_seg = np.sqrt(d_seg_m**2 + l_seg_m**2)
    SLR_seg = np.maximum(SLR_seg, 1e-6)

    # Elevation angle (beta) and depression angle (phi)
    beta_rad = np.arcsin(d_seg_m / SLR_seg)
    beta_deg = np.degrees(beta_rad)
    phi_deg = bank_angle_deg + beta_deg
    phi_rad = np.radians(phi_deg)

    # Engine installation effect (E_WING) for wing-mounted jets
    E_WING = np.zeros_like(phi_deg)
    mask_pos = (phi_deg >= 0.0) & (phi_deg <= 180.0)
    mask_neg = (phi_deg < 0.0) & (phi_deg >= -180.0)

    cos2_phi = np.cos(phi_rad[mask_pos])**2
    sin2_phi = np.sin(phi_rad[mask_pos])**2
    sin2_2phi = np.sin(2.0 * phi_rad[mask_pos])**2
    cos2_2phi = np.cos(2.0 * phi_rad[mask_pos])**2

    num = (0.0039 * cos2_phi + sin2_phi)**0.062
    den = (0.8786 * sin2_2phi + cos2_2phi)
    E_WING[mask_pos] = 10.0 * np.log10(num / den)
    E_WING[mask_neg] = -1.49

    # Ground-to-ground effect (G)
    G = np.full_like(l_seg_m, 10.86)
    mask_G = (l_seg_m >= 0.0) & (l_seg_m <= 914.0)
    G[mask_G] = 11.83 * (1.0 - np.exp(-0.00274 * l_seg_m[mask_G]))

    # Air-to-ground effect (Lambda)
    Lambda = np.zeros_like(beta_deg)
    beta_eff = np.maximum(beta_deg, 0.0)
    mask_L = (beta_eff >= 0.0) & (beta_eff <= 50.0)
    Lambda[mask_L] = 1.137 - (0.0229 * beta_eff[mask_L]) + (9.72 * np.exp(-0.142 * beta_eff[mask_L]))
    # For beta > 50, Lambda remains 0.0

    LA_ADJ = -(E_WING - ((G * Lambda) / 10.86))

    return LA_ADJ, beta_deg
