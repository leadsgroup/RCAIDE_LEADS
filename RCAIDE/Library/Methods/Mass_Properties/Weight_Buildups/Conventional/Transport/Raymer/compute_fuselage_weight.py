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
    """ Calculate the weight of the fuselage of a transport aircraft based on the Raymer method

        Assumptions:
            No fuselage mounted landing gear
            1 cargo door

        Source:
            Aircraft Design: A ConceptualengthApproach (2nd edition)

        Inputs:
            vehicle - data dictionary with vehicle properties                   [dimensionless]
                -.mass_properties.max_takeoff: MTOW                             [kg]
                -.flight_envelope.ultimate_load: ultimate load factor (default: 3.75)
                -.wings['main_wing']: data dictionary with main wing properties
                    -.taper: wing taper ratio
                    -.sweeps.quarter_chord: quarter chord sweep                 [rad]
            fuselage - data dictionary with specific fuselage properties            [dimensionless]
                -.lenghts.total: totalengthlength                                   [m]
                -.width: fuselage width                                         [m]
                -.heights.maximum: maximum height of the fuselage               [m]

        Outputs:
            weight_fuselage - weight of the fuselage                                [kilograms]

        Properties Used:
            N/A
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