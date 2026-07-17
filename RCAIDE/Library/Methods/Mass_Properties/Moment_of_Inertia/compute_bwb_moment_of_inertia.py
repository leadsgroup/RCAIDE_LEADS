# RCAIDE/Library/Methods/Mass_Properties/Moment_of_Inertia/compute_bwb_moment_of_inertia.py 
# 
# Created:  January 2026, S. Shekar, A. Molloy M. Clarke,   

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Blended Wing Body Moment of Inertia
# ----------------------------------------------------------------------------------------------------------------------  
def compute_bwb_moment_of_inertia(bwb_wing, center_of_gravity = [[0, 0, 0]]): 
    ''' computes the moment of inertia tensor for a blended wing body about a given center of gravity.
    Includes the ability to model a  wing fuel tank as a condensed wing
    
    Inputs:
    - Wing 
    - Center of gravity 

    Outputs:
    - bwb wing moment of inertia tensor

    Properties Used:
    N/A
    ''' 
 
    return  bwb_wing.mass_properties.moments_of_inertia.tensor, bwb_wing.mass_properties.mass   