# RCAIDE/Library/Methods/Weights/Correlation_Buildups/BWB/compute_aft_centerbody_weight.py
# 
# Created: Sep 2024, M. Clarke 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units

# ---------------------------------------------------------------------------------------------------------------------- 
# Aft Centerbody Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_aft_centerbody_weight(no_of_engines, aft_centerbody_area, aft_centerbody_taper, TOGW):
    """ Weight estimate for the aft section of a BWB centerbody.
        Regression from FEA by K. Bradley (George Washington University).
        
        Assumptions:
            -The engines are mounted on the aft centerbody
            -The aft centerbody is unpressurized
        
        Sources:
            Bradley, K. R., "A Sizing Methodology for the Conceptual Design of 
            Blended-Wing-Body Transports," NASA/CR-2004-213016, 2004.
            
        Inputs:
            no_of_engines - the number of engines mounted on the aft centerbody 
            [dimensionless]
            aft_centerbody_area - the planform area of the aft centerbody. 
            Typcially the area behind 70% chord [meters**2]
            aft_centerbody_taper - the taper ratio of the aft centerbody (exclude
            the chord taken up by the pressurized passenger cabin) [dimensionless]
            TOGW - Takeoff gross weight of the aircraft [kilograms]
        Outputs:
            W_aft - the estimated structural weight of the BWB aft centerbody
                
        Properties Used:
        N/A
        """         
    # convert to imperial units and shorten variable names 
    S_aft  = aft_centerbody_area  / Units.feet ** 2.0
    l_aft  = aft_centerbody_taper
    W      = TOGW/ Units.pounds
    
    W_aft = (1.0 + 0.05*no_of_engines) * 0.53 * S_aft * (W**0.2) * (l_aft + 0.5)
    
    # convert back to base units
    W_aft = W_aft * Units.pounds
    
    return W_aft