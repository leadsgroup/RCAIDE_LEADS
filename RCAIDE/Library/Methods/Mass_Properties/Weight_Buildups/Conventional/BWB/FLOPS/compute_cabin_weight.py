# RCAIDE/Library/Methods/Weights/Correlation_Buildups/BWB/compute_cabin_weight.py
# 
# Created: Sep 2024, M. Clarke 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 

from RCAIDE.Framework.Core import Units

# ---------------------------------------------------------------------------------------------------------------------- 
#  Cabin Weight 
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_cabin_weight(cabin_area, TOGW):
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
    
    # convert to imperial units
    S_cab    = cabin_area / Units.feet ** 2.0
    W        = TOGW       / Units.pounds
    
    W_cabin = 5.698865 * 0.316422 * (W ** 0.166552) * S_cab ** 1.061158
    
    # convert to SI units
    W_cabin = W_cabin * Units.pounds
    
    return W_cabin