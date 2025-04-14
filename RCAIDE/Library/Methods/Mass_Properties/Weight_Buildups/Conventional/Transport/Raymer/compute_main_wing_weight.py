# RCAIDE/Library/Methods/Weights/Correlation_Buildups/Raymer/compute_main_wing_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE 
from RCAIDE.Framework.Core    import Units

# python imports 
import  numpy as  np
 
# ----------------------------------------------------------------------------------------------------------------------
# Main Wing Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_main_wing_weight(vehicle, wing, settings):
    """
    Calculates the wing weight for transport aircraft using Raymer's empirical method.

    Parameters
    ----------
    vehicle : RCAIDE.Vehicle()
        Vehicle data structure containing:
            - mass_properties.max_takeoff : float
                Maximum takeoff weight [kg]
            - flight_envelope.ultimate_load : float
                Ultimate load factor
            - systems.accessories : str
                Aircraft type ('short-range', 'commuter', 'medium-range',
                'long-range', 'sst', 'cargo')
    wing : RCAIDE.Component()
        Wing component containing:
            - taper : float
                Wing taper ratio
            - sweeps.quarter_chord : float
                Quarter chord sweep angle [rad]
            - thickness_to_chord : float
                Thickness-to-chord ratio
            - aspect_ratio : float
                Wing aspect ratio
            - areas.reference : float
                Wing reference area [m^2]

    Returns
    -------
    weight : float
        Weight of the wing structure [kg]

    Notes
    -----
    This method implements Raymer's correlation for transport aircraft wing
    weight estimation, accounting for geometry, loads, and configuration effects.

    **Major Assumptions**
        * Control surfaces comprise 10% of wing area
        * Correlation based on transport category aircraft data
        * SST configurations treated with zero sweep angle parameter
        * If advanced composites are used, the wing weight is reduced by 10%

    **Theory**
    The wing weight is calculated using:
    .. math::
        W_{wing} = 0.0051(W_{dg}N_z)^{0.557}S_w^{0.649}A^{0.5}(t/c)^{-0.4}(1+\lambda)^{0.1}\cos(\Lambda)^{-1.0}S_{cs}^{0.1}

    where:
        * :math:`W_{dg}` is design gross weight
        * :math:`N_z` is ultimate load factor
        * :math:`S_w` is wing area
        * :math:`A` is aspect ratio
        * :math:`t/c` is thickness ratio
        * :math:`\lambda` is taper ratio
        * :math:`\Lambda` is quarter-chord sweep
        * :math:`S_{cs}` is control surface area

    References
    ----------
    [1] Raymer, D., "Aircraft Design: A Conceptual Approach", AIAA 
        Education Series, 2018. 

    See Also
    --------
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.Raymer.compute_horizontal_tail_weight
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.Raymer.compute_vertical_tail_weight
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.Raymer.compute_operating_empty_weight
    """
    # unpack inputs
    taper   = wing.taper
    sweep   = wing.sweeps.quarter_chord
    area    = wing.areas.reference
    t_c_w   = wing.thickness_to_chord

    Wdg     = vehicle.mass_properties.max_takeoff / Units.lb
    Nz      = vehicle.flight_envelope.ultimate_load
    Sw      = area / Units.ft ** 2
    A       = wing.aspect_ratio
    tc_root = t_c_w
    Scsw    = Sw * .1

    if vehicle.systems.accessories == 'sst':
        sweep = 0
    W_wing = 0.0051 * (Wdg * Nz) ** .557 * Sw ** .649 * A ** .5 * tc_root ** -.4 * (1 + taper) ** .1 * np.cos(
        sweep) ** -1. * Scsw ** .1
    weight = W_wing * Units.lb

    if settings.advanced_composites:
        weight = weight * 0.9

    return weight