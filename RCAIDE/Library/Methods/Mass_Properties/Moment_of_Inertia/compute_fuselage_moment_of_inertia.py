# RCAIDE/Library/Methods/Stability/Moment_of_Inertia/compute_fuselage_moment_of_inertia.py 
# 
# Created:  September 2024, A. Molloy  
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ---------------------------------------------------------------------------------------------------------------------- 
# package imports 
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Fuselage Moment of Intertia
# ----------------------------------------------------------------------------------------------------------------------   
def compute_fuselage_moment_of_inertia(fuselage,center_of_gravity=[[0, 0, 0]]): 
    ''' computes the moment of ienrtia tensor for a generic fuselage about a given center of gravity. 

    Assumptions:
    - Fuselage can be approximated by a hemisphere, cylinder, and cone.
    - Fuselage is hollow with the inner radius being 85% of the outer radius.
    - Fuselage is constant density

    Source:
    [1] Moulton, B. C., and Hunsaker, D. F., “Simplified Mass and Inertial Estimates for Aircraft with Components
    of Constant Density,” AIAA SCITECH 2023 Forum, January 2023, AIAA-2023-2432 DOI: 10.2514/
    6.2023-2432
    
    [2] Weisstein, E. W., "Moment of Inertia -- Cone," Wolfram Research, N.D., https://scienceworld.wolfram.com/physics/MomentofInertiaCone.html 

    Inputs:
    - Fuselage
    - Center of gravity or the point to find the MOI about

    Outputs:
    - Fuselage moment of inertia tensor

    Properties Used:
    N/A
    '''
    
    # ----------------------------------------------------------------------------------------------------------------------   
    # Setup
    # ----------------------------------------------------------------------------------------------------------------------       
    mass_fuselage = 1 * fuselage.mass_properties.mass
    radius_outer  = fuselage.effective_diameter / 2
    radius_inner  = 0.85 * fuselage.effective_diameter / 2 # Assume the inner radius is 85 % of the outer radius    
    center_length = fuselage.lengths.total - fuselage.lengths.nose - fuselage.lengths.tail     # Length of the cylinder (found by subtracting the tail and nose lengths from the total length)
    
    I_total = np.zeros((3, 3)) # Initialize matrix to hold the entire fuselage inertia tensor
    
    # ---------------------------------------------------------------------------------------------------------------------- 
    # Calculate volume fraction of each section
    # ---------------------------------------------------------------------------------------------------------------------- 
    volume_fraction = Volume_Fraction(radius_outer, radius_inner, center_length, fuselage.lengths.tail) # output = [hemisphere, cylinder, cone]
    
    # ----------------------------------------------------------------------------------------------------------------------   
    # Hemisphere
    # ----------------------------------------------------------------------------------------------------------------------   
    origin_hemisphere = np.array([fuselage.lengths.nose, 0, 0]) + np.array(fuselage.origin)
    mass_hemisphere   = mass_fuselage * volume_fraction[0] # mass of the hemisphere
    I                 = np.zeros((3, 3)) # Local inertia tensor 
    
    # Moment of inertia in local system. From Weisstein [2]
    I[0][0] = 2 * mass_hemisphere / 5 *  (radius_outer ** 5 - radius_inner ** 5) /(radius_outer **3 -radius_inner **3) # Ixx
    I[1][1] = 2 * mass_hemisphere / 5 *  (radius_outer ** 5 - radius_inner ** 5) /(radius_outer **3 -radius_inner **3)# Iyy
    I[2][2] = 2 * mass_hemisphere / 5 *  (radius_outer ** 5 - radius_inner ** 5) /(radius_outer **3 -radius_inner **3) # Izz

    # global system
    s        = np.array(center_of_gravity) - np.array(origin_hemisphere)
    I_global = np.array(I) + mass_hemisphere * (np.array(np.dot(s[0], s[0])) * np.array(np.identity(3)) - s*np.transpose(s)) # global inertia tensor for hemisphere
    
    # Add hemisphere to the fuselage inertia tensor
    I_total  = np.array(I_total) + np.array(I_global)
    
    # ----------------------------------------------------------------------------------------------------------------------   
    # cylinder
    # ----------------------------------------------------------------------------------------------------------------------   
    mass_cylinder   = mass_fuselage * volume_fraction[1]
    origin_cylinder = np.array([fuselage.lengths.nose + center_length / 2,0, 0]) + np.array(fuselage.origin) # origin of the cylinder is located a the middle of the cylinder
    
    I =  np.zeros((3, 3))
    
    # Moment of inertia in local system. From Moulton and Hunsaker [1]
    I[0][0]  = mass_cylinder / 2 *  (radius_outer ** 2 + radius_inner ** 2) # Ixx
    I[1][1]  = mass_cylinder / 12 * (3 * (radius_outer ** 2 + radius_inner ** 2) + center_length ** 2) # Iyy
    I[2][2]  = mass_cylinder / 12 * (3 * (radius_outer ** 2 + radius_inner ** 2) + center_length ** 2) # Izz
      
    # transform moment of inertia to global system   
    s        = np.array(center_of_gravity) - np.array(origin_cylinder)
    I_global = np.array(I) + mass_cylinder * (np.array(np.dot(s[0], s[0])) * np.array(np.identity(3)) - s*np.transpose(s))
    
    # Add cylinder to fuselage inertia matrix
    I_total = np.array(I_total) + np.array(I_global)

    # ----------------------------------------------------------------------------------------------------------------------   
    # cone
    # ----------------------------------------------------------------------------------------------------------------------   
    tail_length = fuselage.lengths.tail # length of the cone is defined to be the tail length.
    mass_cone   = mass_fuselage * volume_fraction[2]
    origin_cone = np.array([fuselage.lengths.total - fuselage.lengths.tail,0, 0]) + np.array(fuselage.origin)
    
    I =  np.zeros((3, 3))
  
    # Moment of inertia in local system. From Weisstein [2]. 
    rho     = (mass_cone / (1 / 3 * np.pi * (radius_outer ** 2 * center_length - radius_inner ** 2 * (center_length * radius_inner / radius_outer)))) # density of the cone. Mass divided by volume.
    I[0][0] = rho * (1 / 3 * np.pi * radius_outer ** 2 * tail_length ** 3 + np.pi / 20 * radius_outer ** 4 *tail_length - 1 / 3 * np.pi * radius_inner ** 2 * (tail_length * radius_inner / radius_outer) ** 3 + np.pi / 20 * radius_inner ** 4 *(tail_length * radius_inner / radius_outer))
    I[1][1] = rho * (1 / 3 * np.pi * radius_outer ** 2 * tail_length ** 3 + np.pi / 20 * radius_outer ** 4 *tail_length - 1 / 3 * np.pi * radius_inner ** 2 * (tail_length * radius_inner / radius_outer) ** 3 + np.pi / 20 * radius_inner ** 4 *(tail_length * radius_inner / radius_outer))
    I[2][2] = rho * (np.pi /10 *radius_outer **4 *tail_length -np.pi /10 *radius_inner **4 *(tail_length * radius_inner / radius_outer)) # Izz

    # transform moment of inertia to global system   
    s        = np.array(center_of_gravity) - np.array(origin_cone) # vector from the cone base to the center of gravity of the aircraft. 
    I_global = np.array(I) + mass_cone * (np.array(np.dot(s[0], s[0])) * np.array(np.identity(3)) - s*np.transpose(s))
    
    # Add cone to the fuselage inertia tensor
    I_total = np.array(I_total) + np.array(I_global)
    
    return I_total,  mass_fuselage

def Volume_Fraction(radius_outer, radius_inner, center_length, tail_length):
    '''
    Calculate the volume fraction of each of the three components that make up the entire fuselage
    
    Assumptions:

    Source:

    Inputs:
    - Fuselage dimensions (radii, center length, tail length)

    Outputs:
    - Volume fraction of each fuselage component

    Properties Used:
    N/A
    '''
    # ----------------------------------------------------------------------------------------------------------------------    
    # Individual Component Volumes
    # ----------------------------------------------------------------------------------------------------------------------    
    
    volume_cone       = np.pi * radius_outer ** 2 * tail_length / 3 - np.pi * radius_inner ** 2 * (tail_length * radius_inner / radius_outer) / 3
    volume_hemisphere = 2 / 3 * np.pi * radius_outer ** 3 -2 / 3 * np.pi * radius_inner ** 3 
    volume_cylinder   = np.pi * center_length * (radius_outer ** 2 - radius_inner ** 2)
   
    # ----------------------------------------------------------------------------------------------------------------------    
    # Total Volume
    # ----------------------------------------------------------------------------------------------------------------------        
    volume_total = volume_cone + volume_hemisphere + volume_cylinder
    
    # ----------------------------------------------------------------------------------------------------------------------    
    # Volume Fractions
    # ----------------------------------------------------------------------------------------------------------------------        
    volume_fraction = np.array([volume_hemisphere / volume_total, volume_cylinder / volume_total, volume_cone / volume_total])
    
    return volume_fraction