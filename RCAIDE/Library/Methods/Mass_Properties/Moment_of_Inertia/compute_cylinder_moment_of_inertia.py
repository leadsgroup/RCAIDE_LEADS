# RCAIDE/Library/Methods/Stability/Moment_of_Inertia/compute_cylinder_moment_of_inertia.py 
# 
# Created:  Sept. 2024, A. Molloy  
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# package imports 
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Cylinder Moment of Intertia
# ----------------------------------------------------------------------------------------------------------------------   
def compute_cylinder_moment_of_inertia(origin,mass,length_outer,radius_outer,length_inner = 0,radius_inner = 0,center_of_gravity = np.array([[0,0,0]])):  
    ''' computes the moment of inertia tensor for a hollow cylinder

    Assumptions:
    - Cylinder has a constant density
    - Origin is at the center of the cylinder
    - Cylinder axis of rotation is along the x-axis

    Source:
    [1] Moulton, B. C., and Hunsaker, D. F., “Simplified Mass and Inertial Estimates for Aircraft with Components
    of Constant Density,” AIAA SCITECH 2023 Forum, January 2023, AIAA-2023-2432 DOI: 10.2514/
    6.2023-2432
 
    Inputs:
    - Component properties (origin, mass, length_outer, radius_outer)
    - Center of gravity

    Outputs:
    - cylinder moment of inertia tensor

    Properties Used:
    N/A
    '''
    
    # ----------------------------------------------------------------------------------------------------------------------    
    # Setup
    # ----------------------------------------------------------------------------------------------------------------------           
    I =  np.zeros((3, 3))
    
    # ----------------------------------------------------------------------------------------------------------------------    
    # Moment of inertia in local system. From Moulton and Hunsaker [1]
    # ----------------------------------------------------------------------------------------------------------------------
    
    # Avoid divide by zero error for a point mass
    if  (radius_outer == 0 or length_outer == 0):
        volume = 1
    else:
        volume = (np.pi * radius_outer ** 2 * length_outer - np.pi * radius_inner ** 2 * length_inner) 
    
    rho     = mass / volume
    I[0][0] = rho * (1 / 2 * np.pi * (radius_outer ** 4 * length_outer) - 1 / 2 * np.pi * (radius_inner ** 4 * length_inner)) # Ixx
    I[1][1] = rho * (1 / 12 * (3 *np.pi*(radius_outer ** 4)*length_outer + np.pi * radius_outer ** 2 * length_outer ** 2) - 1 / 12 * (3 * (radius_inner ** 4) *np.pi *length_inner + np.pi * radius_inner ** 2 * length_inner ** 2)) # Iyy
    I[2][2] = rho * (1 / 12 * (3 *np.pi*(radius_outer ** 4)*length_outer + np.pi * radius_outer ** 2 * length_outer ** 2) - 1 / 12 * (3 * (radius_inner ** 4) *np.pi *length_inner + np.pi * radius_inner ** 2 * length_inner ** 2)) # Izz
    
    # ----------------------------------------------------------------------------------------------------------------------    
    # transform moment of inertia to the global system
    # ----------------------------------------------------------------------------------------------------------------------
    s        = np.array(center_of_gravity) - np.array(origin) # Vector between component and the CG    
    I_global = np.array(I) + mass * (np.array(np.dot(s[0], s[0])) * np.array(np.identity(3)) - s*np.transpose(s))
    
    return I_global,  mass