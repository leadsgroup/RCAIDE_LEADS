# RCAIDE/Library/Methods/Mass_Properties/Moment_of_Inertia/compute_fuselage_moment_of_inertia.py 
# 
# Created:  September 2024, A. Molloy  
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ---------------------------------------------------------------------------------------------------------------------- 
# package imports 
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Fuselage Moment of Inertia
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
    outer_radius  = fuselage.effective_diameter / 2
    inner_radius  = 0.85 * fuselage.effective_diameter / 2 # Assume the inner radius is 85 % of the outer radius    
    center_length = fuselage.lengths.total - fuselage.lengths.nose - fuselage.lengths.tail     # Length of the cylinder (found by subtracting the tail and nose lengths from the total length)
    
    I_total = np.zeros((3, 3)) # Initialize matrix to hold the entire fuselage inertia tensor
    
    # ---------------------------------------------------------------------------------------------------------------------- 
    # Calculate volume fraction of each section
    # ---------------------------------------------------------------------------------------------------------------------- 
    volume_fraction = Volume_Fraction(outer_radius, inner_radius, center_length, fuselage.lengths.tail) # output = [hemisphere, cylinder, cone]
    
    # ----------------------------------------------------------------------------------------------------------------------   
    # Hemisphere
    # ----------------------------------------------------------------------------------------------------------------------   
    origin_hemisphere = np.array([fuselage.lengths.nose, 0, 0]) + np.array(fuselage.origin)
    mass_hemisphere   = mass_fuselage * volume_fraction[0] # mass of the hemisphere
    I                 = np.zeros((3, 3)) # Local inertia tensor 
    
    # Moment of inertia in local system. From Weisstein [2]
    I[0][0] = 2 * mass_hemisphere / 5 *  (outer_radius ** 5 - inner_radius ** 5) /(outer_radius **3 -inner_radius **3) # Ixx
    I[1][1] = 2 * mass_hemisphere / 5 *  (outer_radius ** 5 - inner_radius ** 5) /(outer_radius **3 -inner_radius **3)# Iyy
    I[2][2] = 2 * mass_hemisphere / 5 *  (outer_radius ** 5 - inner_radius ** 5) /(outer_radius **3 -inner_radius **3) # Izz
 
    # Add hemisphere to the fuselage inertia tensor
    I_total  += I  
    # ----------------------------------------------------------------------------------------------------------------------   
    # cylinder
    # ----------------------------------------------------------------------------------------------------------------------   
    mass_cylinder   = mass_fuselage * volume_fraction[1] 
    I =  np.zeros((3, 3))
    
    # Moment of inertia in local system. From Moulton and Hunsaker [1]
    I[0][0]  = mass_cylinder / 2 *  (outer_radius ** 2 + inner_radius ** 2) # Ixx
    I[1][1]  = mass_cylinder / 12 * (3 * (outer_radius ** 2 + inner_radius ** 2) + center_length ** 2) # Iyy
    I[2][2]  = mass_cylinder / 12 * (3 * (outer_radius ** 2 + inner_radius ** 2) + center_length ** 2) # Izz
       
    # Add cylinder to fuselage inertia matrix
    I_total += I

    # ----------------------------------------------------------------------------------------------------------------------   
    # cone
    # ----------------------------------------------------------------------------------------------------------------------   
    tail_length = fuselage.lengths.tail # length of the cone is defined to be the tail length.
    mass_cone   = mass_fuselage * volume_fraction[2] 
     
    # Moment of inertia in local system. From Weisstein [2]. 
    rho     = (mass_cone / (1 / 3 * np.pi * (outer_radius ** 2 * center_length - inner_radius ** 2 * (center_length * inner_radius / outer_radius)))) # density of the cone. Mass divided by volume.
    I[0][0] = rho * (1 / 3 * np.pi * outer_radius ** 2 * tail_length ** 3 + np.pi / 20 * outer_radius ** 4 *tail_length - 1 / 3 * np.pi * inner_radius ** 2 * (tail_length * inner_radius / outer_radius) ** 3 + np.pi / 20 * inner_radius ** 4 *(tail_length * inner_radius / outer_radius))
    I[1][1] = rho * (1 / 3 * np.pi * outer_radius ** 2 * tail_length ** 3 + np.pi / 20 * outer_radius ** 4 *tail_length - 1 / 3 * np.pi * inner_radius ** 2 * (tail_length * inner_radius / outer_radius) ** 3 + np.pi / 20 * inner_radius ** 4 *(tail_length * inner_radius / outer_radius))
    I[2][2] = rho * (np.pi /10 *outer_radius **4 *tail_length -np.pi /10 *inner_radius **4 *(tail_length * inner_radius / outer_radius)) # Izz
 
    # Add cone to the fuselage inertia tensor
    I_total += I
    
    # Store moment of inertia tensor on component 
    fuselage.mass_properties.moments_of_inertia.tensor = I_total
    
    return I_total,  mass_fuselage

def Volume_Fraction(outer_radius, inner_radius, center_length, tail_length):
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
    
    volume_cone       = np.pi * outer_radius ** 2 * tail_length / 3 - np.pi * inner_radius ** 2 * (tail_length * inner_radius / outer_radius) / 3
    volume_hemisphere = 2 / 3 * np.pi * outer_radius ** 3 -2 / 3 * np.pi * inner_radius ** 3 
    volume_cylinder   = np.pi * center_length * (outer_radius ** 2 - inner_radius ** 2)
   
    # ----------------------------------------------------------------------------------------------------------------------    
    # Total Volume
    # ----------------------------------------------------------------------------------------------------------------------        
    volume_total = volume_cone + volume_hemisphere + volume_cylinder
    
    # ----------------------------------------------------------------------------------------------------------------------    
    # Volume Fractions
    # ----------------------------------------------------------------------------------------------------------------------        
    volume_fraction = np.array([volume_hemisphere / volume_total, volume_cylinder / volume_total, volume_cone / volume_total])
    
    return volume_fraction