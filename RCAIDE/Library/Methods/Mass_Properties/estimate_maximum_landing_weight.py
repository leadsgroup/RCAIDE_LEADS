# RCAIDE/Methods/Mass_Properties/estimate_maximum_landing_weight.py
# 
# 
# Created:  Sep 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
def estimate_maximum_landing_weight(MTOW):
    """
    Estimates the maximum landing weight (MLW) based on maximum takeoff weight (MTOW).

    Parameters
    ----------
    MTOW : float
        Maximum takeoff weight [kg]

    Returns
    -------
    MLW : float
        Maximum landing weight [kg]

    Notes
    -----
    This function estimates the maximum landing weight using an empirical correlation
    based on maximum takeoff weight. The correlation is derived from statistical
    analysis of commercial aircraft data and provides a reasonable estimate for
    preliminary design purposes.
    
    **Major Assumptions**
        * Empirical correlation is valid for typical commercial aircraft
        * Correlation applies across different aircraft types and sizes
        * No specific operational constraints are considered
    
    **Theory**

    References
    ----------
    [1] Unknown
    """
    MLW = 7E-13 *( MTOW **3) - 7E-07 *( MTOW **2)  + 0.8783 *( MTOW)  + 1601.9 
    return MLW