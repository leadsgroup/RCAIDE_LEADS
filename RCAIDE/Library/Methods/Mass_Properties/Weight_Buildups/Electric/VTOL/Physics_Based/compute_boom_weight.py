# RCAIDE/Library/Methods/Weights/Buildups/Common/compute_boom_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
 
# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
# Compute boom weight
# ----------------------------------------------------------------------------------------------------------------------
def compute_boom_weight(boom,
             maximum_g_load = 3.8,
             safety_factor = 1.5):
    """ Calculates the structural mass of a boom for an eVTOL vehicle, 
        
        Assumptions: 
            Assumes cylindrical boom
        Sources: 

        Inputs:  

        Outputs: 
            weight:                 Estimated Boom Mass             [kg]
        
        Properties Used:
        Material Properties of Imported RCAIDE Solids
    """

    #-------------------------------------------------------------------------------
    # Unpack Inputs
    #------------------------------------------------------------------------------- 
    bLength = boom.lengths.total
    bHeight = boom.heights.maximum 

    #-------------------------------------------------------------------------------
    # Unpack Material Properties
    #-------------------------------------------------------------------------------   
    density     = 1759   # a typical density of carbon fiber is 
    thickness   = 0.01  # thicness of boom is 1 cm

    # Calculate boom area assuming it is a hollow cylinder
    S_wet  = 2* np.pi* (bHeight/2) *bLength + 2*np.pi*(bHeight/2)**2
    weight = S_wet *thickness* density 
    
    return weight