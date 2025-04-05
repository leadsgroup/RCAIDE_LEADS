# RCAIDE/Library/Methods/Stability/Moment_of_Inertia/compute_cuboid_moment_of_inertia.py 
# 
# Created:  September 2024, A. Molloy  
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# package imports 
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Cuboid Moment of Intertia
# ----------------------------------------------------------------------------------------------------------------------   
def compute_cuboid_moment_of_inertia(origin, mass, length_outer, width_outer, height_outer, length_inner = 0, width_inner = 0, height_inner = 0, center_of_gravity = np.array([[0,0,0]])):  
    ''' computes the moment of inertia tensor for a hollow cuboid

    Assumptions:
    - Cuboid has a constant density
    - Origin is at the center of the cuboid
    - length is along the x-axis, width is along the y-axis, height is along the z-axis

    Source:
    [1] Moulton, B. C., and Hunsaker, D. F., “Simplified Mass and Inertial Estimates for Aircraft with Components
    of Constant Density,” AIAA SCITECH 2023 Forum, January 2023, AIAA-2023-2432 DOI: 10.2514/
    6.2023-2432
 
    Inputs:
    - Component properties (origin, mass, lengths, widths, heights)
    - Center of gravity

    Outputs:
    - Cuboid moment of inertia tensor

    Properties Used:
    N/A
    '''
    # ----------------------------------------------------------------------------------------------------------------------    
    # Setup
    # ----------------------------------------------------------------------------------------------------------------------
    I = np.zeros((3, 3)) 
    
    # calcualte volumes
    V2 = length_outer * width_outer * height_outer # Outer volume
    V1 = length_inner * width_inner * height_inner # Inner volume
    
    if  V2 == 0 and V1 == 0:
        temp = 0.000001 # Assigns an arbitrary value to avoid a divide by zero error. This will not affect results as V2 and V1 will be 0
        # Treats object as a point mass
    else:
        temp = (V2 - V1)
    
    # ----------------------------------------------------------------------------------------------------------------------    
    # Calculate inertia tensor. Equations from Moulton adn Hunsaker [1]
    # ----------------------------------------------------------------------------------------------------------------------    
    I[0][0] = mass / 12 * (V2 * (width_outer ** 2 + height_outer ** 2) - V1 * (width_inner ** 2 + height_inner ** 2)) / temp
    I[1][1] = mass / 12 * (V2 * (length_outer ** 2 + height_outer ** 2) - V1 * (length_inner ** 2 + height_inner ** 2)) / temp
    I[2][2] = mass / 12 * (V2 * (length_outer ** 2 + width_outer ** 2) - V1 * (length_inner ** 2 + width_inner ** 2)) / temp
    
    # ----------------------------------------------------------------------------------------------------------------------    
    # transform moment of inertia to the global system
    # ----------------------------------------------------------------------------------------------------------------------
    s        = np.array(center_of_gravity) - np.array(origin) # Vector between component and the CG
    I_global = np.array(I) + mass * (np.array(np.dot(s[0], s[0])) * np.array(np.identity(3)) - s*np.transpose(s))
    
    return I_global,  mass