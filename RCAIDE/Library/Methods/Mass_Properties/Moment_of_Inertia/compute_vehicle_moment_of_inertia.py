# RCAIDE/Library/Methods/Mass_Properties/Moment_of_Inertia/compute_vehicle_moment_of_inertia.py 
# 
# Created:  September 2024, A. Molloy

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_component_moment_of_inertia import compute_component_moment_of_inertia

# python imports
import numpy as  np
# ------------------------------------------------------------------        
#  Component moments of inertia (MOI) tensors
# ------------------------------------------------------------------  
def compute_vehicle_moment_of_inertia(vehicle,overwrite_moment_of_intertia=True, segment = None,verbose=True): 
    ''' sums the moments of inertia of each component in the aircraft. Components summed: fuselages,
    wings (main, horizontal, tail + others), turbofan engines, batteries, motors, batteries, fuel tanks

    Assumptions:
    - All other components than those listed are insignificant

    Source:
 
    Inputs:
    - vehicle
    - Center of gravity

    Outputs:
    - Total aircraft moment of inertia tensor

    Properties Used:
    N/A
    '''    
    if verbose:
        print("\n\n=== COMPONENT MOMENT OF INTERTIA BREAKDOWN REPORT ===" )    
        print("Component \t \t \t Ixx \t \t Iyy  \t \t Izz" )        
     
    # Compute the moment of intertia of all components   
    total_MOI    = np.zeros((3, 3))               
    for key in vehicle.keys():
        item       = vehicle[key]  
        total_MOI  = compute_component_moment_of_inertia(item,vehicle,total_MOI ,segment, verbose)
    
    # print center of gravity  
    if verbose:
        print('\n ***** Aircraft moment of intertia tensor ***** ')
        print(total_MOI) 
 
    # if simulations is part of a mission, store MOI in results vector 
    if segment != None:         
        ones_row  = segment.state.ones_row   
        segment.state.conditions.weights.vehicle.moments_of_inertia_Ixx  = total_MOI[0,0] * ones_row(1)
        segment.state.conditions.weights.vehicle.moments_of_inertia_Ixy  = total_MOI[0,1] * ones_row(1)
        segment.state.conditions.weights.vehicle.moments_of_inertia_Ixz  = total_MOI[0,2] * ones_row(1)
        segment.state.conditions.weights.vehicle.moments_of_inertia_Iyx  = total_MOI[1,0] * ones_row(1)
        segment.state.conditions.weights.vehicle.moments_of_inertia_Iyy  = total_MOI[1,1] * ones_row(1)
        segment.state.conditions.weights.vehicle.moments_of_inertia_Iyz  = total_MOI[1,2] * ones_row(1)
        segment.state.conditions.weights.vehicle.moments_of_inertia_Izx  = total_MOI[2,0] * ones_row(1)
        segment.state.conditions.weights.vehicle.moments_of_inertia_Izy  = total_MOI[2,1] * ones_row(1)
        segment.state.conditions.weights.vehicle.moments_of_inertia_Izz  = total_MOI[2,2] * ones_row(1)        
    
    if overwrite_moment_of_intertia:
        vehicle.mass_properties.moments_of_inertia.tensor = total_MOI 
        
    return total_MOI 