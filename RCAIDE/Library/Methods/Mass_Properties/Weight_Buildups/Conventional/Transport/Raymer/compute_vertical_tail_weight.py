# RCAIDE/Library/Methods/Weights/Correlation_Buildups/Raymer/compute_vertical_tail_weight.py
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
#  Vertical Tail Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_vertical_tail_weight(vehicle, wing, settings):
    """
    Calculates vertical tail weight using Raymer's empirical method.

    Parameters
    ----------
    vehicle : RCAIDE.Vehicle()
        Vehicle data structure containing:
            - mass_properties.max_takeoff : float
                Maximum takeoff weight [kg]
            - flight_envelope.ultimate_load : float
                Ultimate load factor
            - wings['main_wing'] : Data()
                Main wing properties:
                    - origin : array
                        Root location [m]
                    - aerodynamic_center : array
                        Location of aerodynamic center [m]
    wing : RCAIDE.Component()
        Vertical tail component containing:
            - t_tail : bool
                T-tail configuration flag
            - areas.reference : float
                Reference area [m^2]
            - origin : array
                Root location [m]
            - aerodynamic_center : array
                Location of aerodynamic center [m]
            - sweeps.quarter_chord : float
                Quarter chord sweep angle [rad]
            - aspect_ratio : float
                Aspect ratio
            - thickness_to_chord : float
                Thickness-to-chord ratio

    Returns
    -------
    tail_weight : float
        Weight of the vertical tail [kg]

    Notes
    -----
    This method implements Raymer's correlation for transport aircraft vertical
    tail weight estimation, accounting for geometry, loads, and configuration effects.

    **Major Assumptions**
        * T-tail configuration handled through multiplier
        * Correlation based on transport category aircraft data
        * If advanced composites are used, the vertical tail weight is reduced by 15%
    **Theory**
    The vertical tail weight is calculated using:
    .. math::
        W_{vt} = 0.0026(1 + H_t)^{0.225}W_{dg}^{0.556}N_{ult}^{0.536}L_t^{-0.5}S_{vt}^{0.5}K_z^{0.875}\cos(\Lambda)^{-1}AR^{0.35}(t/c)^{-0.5}

    where:
        * :math:`H_t` is T-tail factor (0 or 1)
        * :math:`W_{dg}` is design gross weight
        * :math:`N_{ult}` is ultimate load factor
        * :math:`L_t` is tail arm length
        * :math:`S_{vt}` is vertical tail area
        * :math:`K_z` is vertical tail arm
        * :math:`\Lambda` is quarter-chord sweep
        * :math:`AR` is aspect ratio
        * :math:`t/c` is thickness ratio

    References
    ----------
    [1] Raymer, D., "Aircraft Design: A Conceptual Approach", AIAA 
        Education Series, 2018. 

    See Also
    --------
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.Raymer.compute_horizontal_tail_weight
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.Raymer.compute_main_wing_weight
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.Raymer.compute_operating_empty_weight
    """
    DG          = vehicle.mass_properties.max_takeoff / Units.lbs
    t_tail_flag = wing.t_tail
    wing_origin = wing.origin[0][0] / Units.ft
    wing_ac     = wing.aerodynamic_center[0] / Units.ft
    main_origin = vehicle.wings['main_wing'].origin[0][0] / Units.ft
    main_ac     = vehicle.wings['main_wing'].aerodynamic_center[0] / Units.ft
    Svt         = wing.areas.reference / Units.ft ** 2
    sweep       = wing.sweeps.quarter_chord
    Av          = wing.aspect_ratio
    t_c         = wing.thickness_to_chord 
    Nult        = vehicle.flight_envelope.ultimate_load
    
    H = 0
    if t_tail_flag:
        H = 1
    Lt = (wing_origin + wing_ac - main_origin - main_ac)
    Kz = Lt
    tail_weight = 0.0026 * (1 + H) ** 0.225 * DG ** 0.556 * Nult ** 0.536 \
                  * Lt ** (-0.5) * Svt ** 0.5 * Kz ** 0.875 * np.cos(sweep) ** (-1) * Av ** 0.35 * t_c ** (-0.5)
    if settings.advanced_composites:
        tail_weight *= 0.85
    
    return tail_weight * Units.lbs
