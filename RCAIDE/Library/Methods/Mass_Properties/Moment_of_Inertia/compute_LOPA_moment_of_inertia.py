# RCAIDE/Library/Methods/Stability/Moment_of_Inertia/compute_LOPA_moment_of_inertia.py 
# 
# Created:  September 2024, A. Molloy  
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# package imports 
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  Compute LOPA Moment of Intertia
# ----------------------------------------------------------------------------------------------------------------------   
def compute_LOPA_moment_of_inertia(mass, LOPA, center_of_gravity = np.array([[0,0,0]])):  
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
      
     
    # determine location of passengers (point passes)
    
    # ----------------------------------------------------------------------------------------------------------------------    
    # Calculate inertia tensor. Equations from Moulton adn Hunsaker [1]
    # ----------------------------------------------------------------------------------------------------------------------    
    I[0][0] = mass  
    I[1][1] = mass  
    I[2][2] = mass  
    
    # ----------------------------------------------------------------------------------------------------------------------    
    # transform moment of inertia to the global system
    # ---------------------------------------------------------------------------------------------------------------------- 
    I_global = np.array(I) # + mass * (np.array(np.dot(s[0], s[0])) * np.array(np.identity(3)) - s*np.transpose(s))
    
    return I_global,  mass