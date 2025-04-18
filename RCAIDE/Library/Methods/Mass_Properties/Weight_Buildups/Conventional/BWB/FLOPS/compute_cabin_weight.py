# RCAIDE/Library/Methods/Weights/Correlation_Buildups/BWB/compute_cabin_weight.py
# 
# Created: Sep 2024, M. Clarke 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Geometry.Planform                          import segment_properties  

from copy import deepcopy
# ---------------------------------------------------------------------------------------------------------------------- 
#  Cabin Weight 
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_cabin_weight(vehicle,settings):
    """ Weight estimate for the cabin (forward section of centerbody) of a BWB.
    Regression from FEA by K. Bradley (George Washington University).
    
    Assumptions:
        -The centerbody uses a pressurized sandwich composite structure
        -Ultimate cabin pressure differential of 18.6psi
        -Critical flight condition: +2.5g maneuver at maximum TOGW
    
    Sources:
        Bradley, K. R., "A Sizing Methodology for the Conceptual Design of 
        Blended-Wing-Body Transports," NASA/CR-2004-213016, 2004.
    
    Inputs:
        cabin_area - the planform area representing the passenger cabin  [meters**2]
        TOGW - Takeoff gross weight of the aircraft                      [kilograms]
    Outputs:
        W_cabin - the estimated structural weight of the BWB cabin      [kilograms]
            
    Properties Used:
    N/A
    """
    bwb_vehicle = deepcopy(vehicle)
    bwb_vehicle.wings.main_wing.segments.clear()
    
    for fus_segment in vehicle.wings.main_wing.segments:
        if isinstance(fus_segment, RCAIDE.Library.Components.Fuselages.Segments.Blended_Wing_Segment):
            bwb_fuselage_seg = deepcopy(fus_segment)
            bwb_vehicle.wings.main_wing.segments.append(bwb_fuselage_seg)
            bwb_vehicle.wings.main_wing.spans.projected =  vehicle.wings.main_wing.spans.projected * bwb_fuselage_seg.percent_span_location  
    
    for segment in bwb_vehicle.wings.main_wing.segments:
        segment.percent_span_location  = segment.percent_span_location * (vehicle.wings.main_wing.spans.projected / bwb_vehicle.wings.main_wing.spans.projected )

    bwb_vehicle.wings.main_wing = segment_properties(bwb_vehicle.wings.main_wing,update_ref_areas=True)  
     # convert to imperial units
    A_CB    =  bwb_vehicle.wings.main_wing.areas.reference / Units['feet^2']
    TOGW       =  bwb_vehicle.mass_properties.max_takeoff*9.81/ Units.lbf

    if settings.PRSEUS:
        SR     = bwb_vehicle.wings.main_wing.spans.projected / vehicle.wings.main_wing.spans.projected
        FR = bwb_vehicle.wings.main_wing.spans.projected / bwb_vehicle.wings.main_wing.chords.root
        segments = bwb_vehicle.wings.main_wing.segments
        THETA_CB = (
            sum(segment.sweeps.leading_edge for segment in segments) / len(segments)
            if segments else 0.0
                )/Units.degree
        W_cabin_lbf = 1.06 * 1.20 * 105.95 * (A_CB**0.97) * (TOGW**0.0021) * (FR**-0.75) * (THETA_CB**-0.62) * (SR**-0.0008)
        W_cabin = W_cabin_lbf * Units.lbf / 9.81
    else: 
        W_cabin = 5.698865 * 0.316422 * (TOGW ** 0.166552) * A_CB ** 1.061158
    
    # convert to SI units
    W_cabin = W_cabin * Units.pounds
    
    return W_cabin 