# RCAIDE/Library/Methods/Weights/Correlation_Buildups/FLOPS/compute_vertical_tail_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE 
from RCAIDE.Framework.Core    import Units 
 
# ----------------------------------------------------------------------------------------------------------------------
#  Vertical Tail Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_vertical_tail_weight(vehicle, wing):
    """
    Computes the vertical tail weight using NASA FLOPS weight estimation method.

    Parameters
    ----------
    vehicle : Vehicle
        The vehicle instance containing:
            - mass_properties.max_takeoff : float
                Maximum takeoff weight [kg]
    wing : Wing
        The vertical tail instance containing:
            - taper : float
                Taper ratio
            - areas.reference : float
                Planform area [m²]

    Returns
    -------
    WVT : float
        Vertical tail weight [kg]

    Notes
    -----
    Uses FLOPS correlation developed from transport aircraft database. The method accounts 
    for tail size, aircraft weight, and planform effects.

    **Major Assumptions**
        * Conventional tail configuration
        * Single vertical tail (NVERT = 1). Multiple vertical tails are computed separately and summed.

    **Theory**
    Weight is computed using:
    .. math::
        W_{VT} = 0.32W_{TO}^{0.3}(\\lambda_{VT} + 0.5)N_{VT}^{0.7}S_{VT}^{0.85}

    where:
        * W_TO = takeoff gross weight
        * λ_VT = vertical tail taper ratio
        * N_VT = number of vertical tails (fixed at 1)
        * S_VT = vertical tail area

    The correlation accounts for:
        * Bending and torsional stiffness requirements
        * Rudder integration
        * Attachment structure
        * Standard design margins
        * Directional stability requirements

    References
    ----------
    [1] NASA Flight Optimization System (FLOPS)

    See Also
    --------
    RCAIDE.Library.Methods.Weights.Correlation_Buildups.FLOPS.compute_horizontal_tail_weight
    """
    DG          = vehicle.mass_properties.max_takeoff / Units.lbs  # Design gross weight in lb
    TRVT        = wing.taper
    NVERT       = 1  # Number of vertical tails
    WVT         = 0.32 * DG ** 0.3 * (TRVT + 0.5) * NVERT ** 0.7 * (wing.areas.reference/Units.ft**2)**0.85
    return WVT * Units.lbs


