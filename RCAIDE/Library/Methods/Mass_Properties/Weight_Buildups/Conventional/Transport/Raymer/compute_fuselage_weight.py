# RCAIDE/Library/Methods/Weights/Correlation_Buildups/Raymer/compute_fuselage_weight.py
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
# fuselage Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_fuselage_weight(vehicle, fuselage, settings):
    """
    Calculates the weight of the fuselage for transport aircraft using Raymer's method.

    Parameters
    ----------
    vehicle : RCAIDE.Vehicle()
        Vehicle data structure containing:
            - mass_properties.max_takeoff : float
                Maximum takeoff weight [kg]
            - flight_envelope.ultimate_load : float
                Ultimate load factor (default: 3.75)
            - wings['main_wing'] : Data()
                Main wing properties including taper and sweep
    fuselage : RCAIDE.Component()
        Fuselage component containing:
            - lengths.total : float
                Total fuselage length [m]
            - width : float
                Maximum fuselage width [m]
            - heights.maximum : float
                Maximum fuselage height [m]
    settings : Data()
        Configuration settings containing:
            - Raymer.fuselage_mounted_landing_gear_factor : float
                Factor for fuselage-mounted landing gear
            - Raymer.cargo_doors_number : int
                Number of cargo doors
            - Raymer.cargo_doors_clamshell : bool
                True if cargo doors are clamshell doors, False otherwise

    Returns
    -------
    weight_fuselage : float
        Weight of the fuselage structure [kg]

    Notes
    -----
    This method implements Raymer's semi-empirical correlation for transport aircraft
    fuselage weight estimation. The correlation accounts for size, loads, and wing-body
    intersection effects. It also accounts for the number of cargo doors and their type (if present).

    **Major Assumptions**
        * No fuselage-mounted landing gear by default
        * One cargo door (Kdoor = 1.06)
        * Correlation based on transport category aircraft data

    **Theory**
    The fuselage weight is calculated using:
    .. math::
        W_{fus} = 0.328K_{door}K_{lg}(W_{dg}N_{ult})^{0.5}L^{0.25}S_f^{0.302}(1+K_{ws})^{0.04}(L/D)^{0.1}

    where:
        * :math:`K_{ws}` accounts for wing-body intersection effects
        * :math:`S_f` is the fuselage wetted area
        * :math:`L/D` is the fuselage fineness ratio

    References
    ----------
    [1] Raymer, D., "Aircraft Design: A Conceptual Approach", AIAA 
        Education Series, 2018. 

    See Also
    --------
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.Raymer.compute_main_wing_weight
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.Raymer.compute_operating_empty_weight
    """
    Klg         = settings.fuselage_mounted_landing_gear_factor
    DG          = vehicle.mass_properties.max_takeoff / Units.lbs
    length      = fuselage.lengths.total/ Units.ft
    fuselage_w  = fuselage.width / Units.ft
    fuselage_h  = fuselage.heights.maximum / Units.ft
    
    if settings.cargo_doors_number == 0:
        if settings.cargo_doors_clamshell:
            Kdoor       = 1.12  # clamshell cargo door
        else:
            Kdoor       = 1.  # 0 cargo door
    if settings.cargo_doors_number == 1:
        Kdoor       = 1.06  # 1 cargo door
    if settings.cargo_doors_number == 2:
        if settings.cargo_doors_clamshell:
            Kdoor       = 1.25  # clamshell cargo door
        else:
            Kdoor       = 1.12  # 2 cargo door
   

    D           = (fuselage_w + fuselage_h) / 2.
    Sf          = np.pi * (length/ D - 1.7) * D ** 2  # fuselage wetted area, ft**2
    wing        = vehicle.wings['main_wing']
    Kws         = 0.75 * (1 + 2 * wing.taper) / (1 + wing.taper) * (wing.spans.projected / Units.ft *
                                                            np.tan(wing.sweeps.quarter_chord)) / length

    weight_fuselage = 0.328 * Kdoor * Klg * (DG * vehicle.flight_envelope.ultimate_load) ** 0.5 * length** 0.25 * \
                 Sf ** 0.302 * (1 + Kws) ** 0.04 * (length/ D) ** 0.1
    
    if settings.advanced_composites:
        weight_fuselage *= 0.925

    return weight_fuselage * Units.lbs