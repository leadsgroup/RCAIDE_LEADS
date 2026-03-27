# RCAIDE/Library/Methods/Mass_Properties/Moment_of_Intertia/sum_component_moments_of_inertia.py 
# 
# Created:  Jul 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
import RCAIDE
from RCAIDE.Library.Components                                 import Component

# python import
import numpy as np
# ----------------------------------------------------------------------------------------------------------------------
#  Recursive MOI
# ----------------------------------------------------------------------------------------------------------------------   
def compute_component_moment_of_inertia(moment_of_inertia_df,component,vehicle,total_MOI,segment=None,verbose=True):
    """ Recursively computes the compute moment of inertia all components and subcomponents

    Assumptions:
    None

    Source:
    N/A

    Inputs:
       compoment

    Outputs:
       None
    """      
    vehicle_CG = vehicle.mass_properties.center_of_gravity
    if isinstance(component,Component.Container): 
        for key in component.keys():
            item = component[key]        
            total_MOI = compute_component_moment_of_inertia(moment_of_inertia_df,item,vehicle,total_MOI,segment,verbose)
    if isinstance(component,Component):
        component.compute_moments_of_inertia(vehicle, center_of_gravity=vehicle_CG)
        update_total_moment_of_inertia(total_MOI,vehicle_CG,component,segment, verbose, moment_of_inertia_df)  
        for key in component.keys():
            item = component[key]
            if isinstance(item,Component.Container):
                total_MOI = compute_component_moment_of_inertia(moment_of_inertia_df,item,vehicle,total_MOI,segment,verbose)
            if isinstance(item,Component):
                item.compute_moments_of_inertia(vehicle, center_of_gravity=vehicle_CG)
                update_total_moment_of_inertia(total_MOI,vehicle_CG,item,segment, verbose,moment_of_inertia_df) 
        
    return total_MOI
 
def update_total_moment_of_inertia(total_MOI,vehicle_CG,C,segment,verbose,moment_of_inertia_df):
    # compoment MOI
    I_component     = C.mass_properties.moments_of_inertia.tensor 
    component_mass  = C.mass_properties.mass
    
    # MOI due to parallel axis theorm
    component_CG    = np.array(C.mass_properties.center_of_gravity) + np.array(C.origin)   
    s               = np.array(vehicle_CG) -  np.array(component_CG)
    I_parallel_axis = component_mass * (np.array(np.dot(s[0], s[0])) * np.array(np.identity(3)) - np.outer(s,s))
    
    # total moment of inertia 
    I_global        = I_component + I_parallel_axis
    
    total_MOI      += I_global
    
    if verbose:
        name_column_width = 20
        num_column_width  = 6
        print(f"{C.tag.ljust(name_column_width)}",'\t \t', f"{str(round(I_global[0][0],2)).ljust(num_column_width)}", '\t', f"{str(round(I_global[1][1],2)).ljust(num_column_width)}", '\t'f"{str(round(I_global[2][2],2)).ljust(num_column_width)}", '\t'  )    
    moment_of_inertia_df.loc[len(moment_of_inertia_df)] = [
    C.tag,
    round(C.mass_properties.mass, 2),
    round(I_global[0][0], 2),
    round(I_global[1][1], 2),
    round(I_global[2][2], 2),
    round(I_global[0][1], 2),
    round(I_global[0][2], 2),
    round(I_global[1][2], 2),
    ]
    if segment != None:
        ones_row  = segment.state.ones_row  
        segment.state.conditions.weights.components.moments_of_inertia_Ixx[C.tag] = I_global[0][0]  * ones_row(1) 
        segment.state.conditions.weights.components.moments_of_inertia_Ixy[C.tag] = I_global[0][1]  * ones_row(1)
        segment.state.conditions.weights.components.moments_of_inertia_Ixz[C.tag] = I_global[0][2]  * ones_row(1)
        segment.state.conditions.weights.components.moments_of_inertia_Iyx[C.tag] = I_global[1][0]  * ones_row(1)
        segment.state.conditions.weights.components.moments_of_inertia_Iyy[C.tag] = I_global[1][1]  * ones_row(1)
        segment.state.conditions.weights.components.moments_of_inertia_Iyz[C.tag] = I_global[1][2]  * ones_row(1)
        segment.state.conditions.weights.components.moments_of_inertia_Izx[C.tag] = I_global[2][0]  * ones_row(1)
        segment.state.conditions.weights.components.moments_of_inertia_Izy[C.tag] = I_global[2][1]  * ones_row(1)
        segment.state.conditions.weights.components.moments_of_inertia_Izz[C.tag] = I_global[2][2]  * ones_row(1)   
    return total_MOI
