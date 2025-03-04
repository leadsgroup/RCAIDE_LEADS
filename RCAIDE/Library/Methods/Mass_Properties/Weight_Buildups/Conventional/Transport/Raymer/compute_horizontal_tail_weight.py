# RCAIDE/Library/Methods/Weights/Correlation_Buildups/Raymer/compute_horizontal_tail_weight.py
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE
import RCAIDE 
from RCAIDE.Framework.Core    import Units

# python imports 
import  numpy as  np
 
# ----------------------------------------------------------------------------------------------------------------------
#  Horizontal Tail Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_horizontal_tail_weight(vehicle, wing, settings,elevator_fraction=0.4):
    """ Calculates horizontal tail weight based on Raymer method

        Assumptions:
            If all-moving horizontal tail, change Kuht to 1.143
        Source:
            Aircraft Design: A Conceptual Approach

        Inputs:
            vehicle - data dictionary with vehicle properties                    [dimensionless]
                -.mass_properties.max_takeoff: MTOW                             [kilograms]
                -.flight_envelope.ultimate_load: ultimate load factor (default: 3.75)
                -.wings['main_wing']: data dictionary with properties of main wing
                    -.aerodynamic_center: aerodynamic center as measured from root leading edge
                    -.origin: root of main wing as measured from nose of aircraft
                -.fuselages['fuselage'].width: width of the fuselage
            wing    - data dictionary with specific tail properties              [dimensionless]
                -.areas.reference: tail surface area                            [m^2}
                -.origin: location of tail measured from nose
                -.aerodynamic_center: location of ac measured from leading edge
                -.sweeps.quarter_chord: quarter chord sweep of tail             [rad]
                -.thickness_to_chord: t/c of tail
                -.span.projected: project span of tail                          [m]
                -.aspect_ratio: aspect ratio of wing
            elevator_fraction - fraction of horizontal tail for elevator = 0.4

        Outputs:
            tail_weight: horizontal tail weight                                [kilograms]

        Properties Used:
            N/A
    """

    ref_wing = None 
    for wing in  vehicle.wings:
        if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
            ref_wing  =  wing
    
    S = 0
    if ref_wing == None:
        for wing in  vehicle.wings:
            if S < wing.areas.reference:
                ref_wing = wing
    
    L_fuselage = 0
    for fuselage in vehicle.fuselages:
        if L_fuselage < fuselage.lengths.total:
            ref_fuselage = fuselage
            L_fuselage = ref_fuselage.lengths.total
    diff =[]
    names = []
    horiz_tail_location = vehicle.wings.horizontal_stabilizer.origin[0][0]
    for segment in ref_fuselage.segments:
        segment_loc = segment.percent_x_location * ref_fuselage.lengths.total
        diff.append(np.abs(segment_loc - horiz_tail_location))
        names.append(segment.tag)
    min_value = min(diff)
    min_index = diff.index(min_value)
            
    Kuht    = 1 # not a all-moving unit horizontal tail
    Fw      = ref_fuselage.segments[names[min_index]].width / Units.ft
    Bh      = wing.spans.projected / Units.ft
    DG      = vehicle.mass_properties.max_takeoff / Units.lbs
    Sht     = wing.areas.reference / Units.ft ** 2
    Lt      = (wing.origin[0][0] + wing.aerodynamic_center[0] - ref_wing.origin[0][0] -
                ref_wing.aerodynamic_center[0]) / Units.ft
    Ky      = 0.3 * Lt
    sweep   = wing.sweeps.quarter_chord
    Ah      = wing.aspect_ratio
    Se      = elevator_fraction * Sht

    tail_weight = 0.0379 * Kuht * (1 + Fw / Bh) ** (-0.25) * DG ** 0.639 *\
                  vehicle.flight_envelope.ultimate_load ** 0.1 * Sht ** 0.75 * Lt ** -1 *\
                  Ky ** 0.704 * np.cos(sweep) ** (-1) * Ah ** 0.166 * (1 + Se / Sht) ** 0.1
    
    if settings.advanced_composites:
        tail_weight *= 0.85

    return tail_weight * Units.lbs