# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/compute_zero_lift_moment.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  compute_zero_lift_moment
# ----------------------------------------------------------------------------------------------------------------------
def compute_zero_lift_moment(CLift_plus, CLift_minus, CM_plus, CM_minus):
    """
    Pitching moment coefficient at zero lift from cases at equal and opposite angles of attack.

    With linear lift and moment, Cm(alpha) = Cm(0) + Cm_alpha alpha evaluated at the zero-lift angle
    alpha_0L = -C_L0 / C_L_alpha gives Cm_0L = Cm(0) - C_L0 Cm_alpha / C_L_alpha; the perturbation cancels in the
    ratio of the two slopes.

    Parameters
    ----------
    CLift_plus, CLift_minus : array
        Lift coefficients at +delta and -delta
    CM_plus, CM_minus : array
        Pitching moment coefficients at +delta and -delta

    Returns
    -------
    CM_zero_lift : array

    References
    ----------
    [1] Lutze, F. H., "Stability and Control: Some Characteristics of Lifting Surfaces, and Pitch-Moments,"
        AOE 3134 course notes, Virginia Tech (pitch-moment at zero lift; C_L0 = -a alpha_0L)
    """
    CLift_0 = (CLift_plus + CLift_minus) / 2
    CM_0    = (CM_plus + CM_minus) / 2
    return CM_0 - CLift_0 * (CM_plus - CM_minus) / (CLift_plus - CLift_minus)
