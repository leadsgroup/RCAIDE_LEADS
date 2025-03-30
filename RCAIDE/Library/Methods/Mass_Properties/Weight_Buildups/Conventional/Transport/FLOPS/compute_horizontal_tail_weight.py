# RCAIDE/Library/Methods/Weights/Correlation_Buildups/FLOPS/compute_horizontal_tail_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE 
from RCAIDE.Framework.Core    import Units
 
# ----------------------------------------------------------------------------------------------------------------------
#  Horizontal Tail Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_horizontal_tail_weight(vehicle, wing): 
    """
    Computes the horizontal tail weight using NASA FLOPS weight estimation method.

    Parameters
    ----------
    vehicle : Vehicle
        The vehicle instance containing:
            - mass_properties.max_takeoff : float
                Maximum takeoff weight [kg]
    wing : Wing
        The horizontal tail instance containing:
            - areas.reference : float
                Planform area [m²]
            - taper : float
                Taper ratio

    Returns
    -------
    WHT : float
        Horizontal tail weight [kg]

    Notes
    -----
    Uses FLOPS correlation developed from transport aircraft database. The method accounts 
    for tail size, aircraft weight, and planform effects.

    **Major Assumptions**
        * Conventional tail configuration
        * All-moving or fixed stabilizer with elevator
        * Standard aluminum construction
        * Conventional aerodynamic loads
        * No special design features (e.g., folding)

    **Theory**
    Weight is computed using:
    .. math::
        W_{HT} = 0.53 S_{HT} (TOGW)^{0.2} (\\lambda_{HT} + 0.5)

    where:
        * S_HT = horizontal tail area
        * TOGW = takeoff gross weight
        * λ_HT = horizontal tail taper ratio

    The correlation accounts for:
        * Bending and torsional stiffness requirements
        * Control surface integration
        * Attachment structure
        * Standard design margins

    References
    ----------
    [1] NASA Langley Research Center, "Flight Optimization System, Release 8.23, 
        User's Guide", 2011.

    See Also
    --------
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.FLOPS.compute_vertical_tail_weight
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.FLOPS.compute_operating_empty_weight
    """
    SHT     = wing.areas.reference / Units.ft **2
    DG      = vehicle.mass_properties.max_takeoff / Units.lbs
    TRHT    = wing.taper
    WHT     = 0.53 * SHT * DG ** 0.2 * (TRHT + 0.5)
    return WHT * Units.lbs
