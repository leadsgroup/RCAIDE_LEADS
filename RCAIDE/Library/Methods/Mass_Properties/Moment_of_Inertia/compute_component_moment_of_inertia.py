# RCAIDE/Library/Methods/Mass_Properties/Moment_of_Intertia/sum_component_moments_of_inertia.py 
# 
# Created:  Jul 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
import RCAIDE
from RCAIDE.Library.Components                                 import Component   

# ----------------------------------------------------------------------------------------------------------------------
#  Recursive MOI
# ----------------------------------------------------------------------------------------------------------------------   
def compute_component_moment_of_inertia(component,vehicle,total_MOI,segment=None,verbose=True):
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

    if isinstance(component,Component.Container): 
        for key in component.keys():
            item = component[key]        
            total_MOI = compute_component_moment_of_inertia(item,vehicle,total_MOI,segment,verbose)
    if isinstance(component,Component):
        component.compute_moments_of_inertia(vehicle, center_of_gravity=vehicle.mass_properties.center_of_gravity)
        update_total_moment_of_inertia(total_MOI,component,segment, verbose)
        for key in component.keys():
            item = component[key]
            if isinstance(item,Component.Container):
                total_MOI = compute_component_moment_of_inertia(item,vehicle,total_MOI,segment,verbose)
            if isinstance(item,Component):
                item.compute_moments_of_inertia(vehicle, center_of_gravity=vehicle.mass_properties.center_of_gravity)
                update_total_moment_of_inertia(total_MOI,item,segment, verbose) 
        
    return total_MOI
 
def update_total_moment_of_inertia(total_MOI,C,segment,verbose):  
    I             = C.mass_properties.moments_of_inertia.tensor
    total_MOI    += I
     
    if verbose:
        name_column_width = 20
        num_column_width  = 6
        print(f"{C.tag.ljust(name_column_width)}",'\t \t', f"{str(round(I[0][0],2)).ljust(num_column_width)}", '\t', f"{str(round(I[1][1],2)).ljust(num_column_width)}", '\t'f"{str(round(I[2][2],2)).ljust(num_column_width)}", '\t'  )    
    if segment != None:
        ones_row  = segment.state.ones_row  
        segment.state.conditions.weights.components.moments_of_inertia_Ixx[C.tag] = I[0][0]  * ones_row(1) 
        segment.state.conditions.weights.components.moments_of_inertia_Ixy[C.tag] = I[0][1]  * ones_row(1)
        segment.state.conditions.weights.components.moments_of_inertia_Ixz[C.tag] = I[0][2]  * ones_row(1)
        segment.state.conditions.weights.components.moments_of_inertia_Iyx[C.tag] = I[1][0]  * ones_row(1)
        segment.state.conditions.weights.components.moments_of_inertia_Iyy[C.tag] = I[1][1]  * ones_row(1)
        segment.state.conditions.weights.components.moments_of_inertia_Iyz[C.tag] = I[1][2]  * ones_row(1)
        segment.state.conditions.weights.components.moments_of_inertia_Izx[C.tag] = I[2][0]  * ones_row(1)
        segment.state.conditions.weights.components.moments_of_inertia_Izy[C.tag] = I[2][1]  * ones_row(1)
        segment.state.conditions.weights.components.moments_of_inertia_Izz[C.tag] = I[2][2]  * ones_row(1)   
    return total_MOI
