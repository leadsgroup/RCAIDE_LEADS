# RCAIDE/Library/Methods/Mass_Properties/Moment_of_Inertia/compute_cuboid_moment_of_inertia.py 
# 
# Created:  September 2024, A. Molloy  
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE

# # package imports 
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Cuboid Moment of Inertia
# ----------------------------------------------------------------------------------------------------------------------   
def compute_cuboid_moment_of_inertia(component,outer_length, outer_width, outer_height,
                                     inner_length = 0, inner_width = 0, inner_height = 0,
                                     center_of_gravity = np.array([[0,0,0]]),fuel_tank=False):  
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
    # unpack 
    # ---------------------------------------------------------------------------------------------------------------------- 
    mass   = component.mass_properties.mass
    
    # ----------------------------------------------------------------------------------------------------------------------    
    # Setup
    # ----------------------------------------------------------------------------------------------------------------------
    I = np.zeros((3, 3)) 
    
    # calcualte volumes
    V2 = outer_length * outer_width * outer_height # Outer volume
    V1 = inner_length * inner_width * inner_height # Inner volume
    
    if  V2 == V1:
        temp = 0.000001 # Assigns an arbitrary value to avoid a divide by zero error. This will not affect results as V2 and V1 will be 0
        # Treats object as a point mass
    else:
        temp = (V2 - V1)
    
    # ----------------------------------------------------------------------------------------------------------------------    
    # Calculate inertia tensor. Equations from Moulton and Hunsaker [1]
    # ----------------------------------------------------------------------------------------------------------------------    
    I[0][0] = mass / 12 * (V2 * (outer_width ** 2 + outer_height ** 2) - V1 * (inner_width ** 2 + inner_height ** 2)) / temp
    I[1][1] = mass / 12 * (V2 * (outer_length ** 2 + outer_height ** 2) - V1 * (inner_length ** 2 + inner_height ** 2)) / temp
    I[2][2] = mass / 12 * (V2 * (outer_length ** 2 + outer_width ** 2) - V1 * (inner_length ** 2 + inner_width ** 2)) / temp
    
    # Store moment of inertia tensor on component 
    component.mass_properties.moments_of_inertia.tensor                 = I 
    component.mass_properties.moments_of_inertia.non_dimensional_tensor = I / mass
    
    if fuel_tank == True:
        # unpack fuel 
        fuel      = component.fuel
        
        # compute MOI of fuel assume inner walls of tank is boundary of fuel
        _,_ = compute_cuboid_moment_of_inertia(fuel,inner_length, inner_width, inner_height,center_of_gravity=center_of_gravity,fuel_tank=False)        
        
       
    return I ,  mass