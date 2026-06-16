# RCAIDE/Library/Methods/Mass_Properties/Moment_of_Inertia/compute_wing_moment_of_inertia.py 
# 
# Created:  September 2023, A. Molloy  
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE 

# package imports 
import numpy as np  
# ----------------------------------------------------------------------------------------------------------------------
#  Compute Wing Moment of Inertia
# ----------------------------------------------------------------------------------------------------------------------  
def compute_wing_integral_tank_moment_of_inertia(fuel_tank,wing, center_of_gravity = [[0, 0, 0]]):
    ''' computes the moment of inertia tensor for an integral tank within wing about a given center of gravity. 

    Assumptions:
    - Fuel tank walls are not considered in MOI calculation
    - Fuel is uniformly distributed in tank
    - Fuel in tank has a constant density

    Source:
    [1] Moulton, B. C., and Hunsaker, D. F., “Simplified Mass and Inertial Estimates for Aircraft with Components
    of Constant Density,” AIAA SCITECH 2023 Forum, January 2023, AIAA-2023-2432 DOI: 10.2514/
    6.2023-2432
    
    [2] Fuel tank references: These were used to estimate the length percentages. 
    - https://assets.publishing.service.gov.uk/media/5422fa1aed915d13710007a1/2-2007_G-YMME.pdf
    - https://oat.aero/2023/03/17/airbus-a380-general-familiarisation-fuel-storage/
    - http://www.b737.org.uk/fuel.htm
    - https://slideplayer.com/slide/3854059/
    
    Inputs:
    - Fuel Tank
    - Wing 
    - Center of gravity 

    Outputs:
    - wing moment of inertia tensor

    Properties Used:
    N/A
    '''   
    mass         = fuel_tank.fuel.mass_properties.mass 
    I_local_fuel = fuel_tank.fuel.mass_properties.moments_of_inertia.tensor  
    I_local_fuel_non_dim = I_local_fuel / mass

    # intialize matrices  
    I_local_tank        = np.zeros((3, 3))
    I_local_tank_non_dim =np.zeros((3, 3)) 
        
    # Store moment of inertia tensors of tank and fuel 
    fuel_tank.fuel.mass_properties.moments_of_inertia.tensor                 = I_local_fuel
    fuel_tank.fuel.mass_properties.moments_of_inertia.non_dimensional_tensor = I_local_fuel_non_dim
    fuel_tank.mass_properties.moments_of_inertia.tensor                      = I_local_tank
    fuel_tank.mass_properties.moments_of_inertia.non_dimensional_tensor      = I_local_tank_non_dim 
    
    return I_local_fuel,  mass
    