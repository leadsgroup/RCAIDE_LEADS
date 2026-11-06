# RCAIDE/Library/Methods/Stability/Common/compute_lateral_derivatives.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_lateral_derivatives
# ----------------------------------------------------------------------------------------------------------------------
def compute_lateral_derivatives(state, settings, vehicle):
    """
    Roll and yaw rate derivatives with respect to stability-axis rates. The VLM reports forces and moments in stability
    axes against rates about body axes, so with p_b = p_s cos(alpha) - r_s sin(alpha) and
    r_b = p_s sin(alpha) + r_s cos(alpha) from Etkin & Reid (B.12,2):
    dC/dp_s = dC/dp_b cos(alpha) + dC/dr_b sin(alpha) and dC/dr_s = -dC/dp_b sin(alpha) + dC/dr_b cos(alpha).

    Parameters
    ----------
    state : Segment or State
        conditions and analyses.aerodynamics (for user overrides in stability_derivatives)
    settings : Data
    vehicle : Vehicle

    Returns
    -------
    None
        Rewrites CY_p, CL_p, CN_p, CY_r, CL_r, CN_r in conditions.static_stability.derivatives

    References
    ----------
    [1] Etkin, B., and Reid, L. D., Dynamics of Flight: Stability and Control, 3rd ed., Wiley, 1996, Appendix B.12.
    """
    conditions  = state.conditions
    derivatives = conditions.static_stability.derivatives
    alpha       = conditions.aerodynamics.angles.alpha
    overrides   = state.analyses.aerodynamics.stability_derivatives
    for coefficient in ('CY', 'CL', 'CN'):
        C_p, C_r = derivatives[coefficient + '_p'], derivatives[coefficient + '_r']
        rotated  = {'_p': C_p * np.cos(alpha) + C_r * np.sin(alpha), '_r': -C_p * np.sin(alpha) + C_r * np.cos(alpha)}
        for suffix, value in rotated.items():
            override = getattr(overrides, coefficient + suffix, None)
            derivatives[coefficient + suffix] = value if override is None else override * np.ones_like(alpha)
    return
