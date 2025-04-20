# RCAIDE/Library/Methods/Weights/Correlation_Buildups/BWB/compute_cabin_weight.py
# 
# Created: Sep 2024, M. Clarke 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE
from RCAIDE.Framework.Core import Units  
import  numpy as  np
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
        ***** ADD SOURCE********
    
    Inputs:
        cabin_area - the planform area representing the passenger cabin  [meters**2]
        TOGW - Takeoff gross weight of the aircraft                      [kilograms]
    Outputs:
        W_cabin - the estimated structural weight of the BWB cabin      [kilograms]
            
    Properties Used:
    N/A
    """ 
    
    fus_seg_max_percent_span = 0
    root_chord               = 0
    span                     = 0
    segment_le_sweeps        = []
    for wing in  vehicle.wings:
        span =  np.maximum(span,wing.spans.projected )
        root_chord =  np.maximum(root_chord,wing.chords.root)
        for segment in wing:
            if isinstance(segment, RCAIDE.Library.Components.Fuselages.Segments.Blended_Wing_Segment):
                fus_seg_max_percent_span =  np.maximum(fus_seg_max_percent_span,segment.percent_span_location  )
                segment_le_sweeps.append(segment.sweeps.leading_edge/Units.degree)  
    
     # convert to imperial units
    A_CB     =  vehicle.reference_area / Units['feet^2']
    TOGW     =  vehicle.mass_properties.max_takeoff*9.81/ Units.lbf
    THETA_CB = np.array(segment_le_sweeps ) / len(segment_le_sweeps)

    if settings.PRSEUS:
        SR  = fus_seg_max_percent_span
        FR  = fus_seg_max_percent_span*span/ root_chord   
        W_cabin_lbf = 1.06 * 1.20 * 105.95 * (A_CB**0.97) * (TOGW**0.0021) * (FR**-0.75) * (THETA_CB**-0.62) * (SR**-0.0008)
        W_cabin     = W_cabin_lbf * Units.lbf / 9.81
    else: 
        W_cabin = 5.698865 * 0.316422 * (TOGW ** 0.166552) * A_CB ** 1.061158
    
    # convert to SI units
    W_cabin = W_cabin * Units.pounds
    
    return W_cabin 