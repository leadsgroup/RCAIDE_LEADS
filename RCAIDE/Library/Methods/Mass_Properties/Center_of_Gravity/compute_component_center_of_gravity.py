# RCAIDE/Library/Methods/Mass_Properties/Center_of_Gravity/compute_component_center_of_gravity.py 
# 
# Created:  Dec 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports 
import RCAIDE
from RCAIDE.Library.Components                                 import Component    

# python imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Recursive C.G.
# ----------------------------------------------------------------------------------------------------------------------   
def compute_component_center_of_gravity(component,vehicle,total_mass,total_moment,segment=None,verbose=True):
    """ Recursively computes the center of gravity all components and subcomponents

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
            total_mass,total_moment = compute_component_center_of_gravity(item,vehicle,total_mass,total_moment,segment,verbose)
    if isinstance(component,Component):
        component.compute_center_of_gravity(vehicle)
        update_mass_and_moment(total_mass,total_moment,component,segment,verbose)     
        for key in component.keys():
            item = component[key]
            if isinstance(item,Component.Container):
                total_mass,total_moment = compute_component_center_of_gravity(item,vehicle,total_mass,total_moment,segment,verbose)
            if isinstance(item,Component):
                item.compute_center_of_gravity(vehicle)
                update_mass_and_moment(total_mass,total_moment,item,segment,verbose)   
    return total_mass,total_moment
 

def update_mass_and_moment(total_mass,total_moment,C,segment,verbose):  
    global_cg_loc = np.array(C.mass_properties.center_of_gravity) + np.array(C.origin)  
    if C.xz_plane_symmetric:
        global_cg_loc[0][1] = 0 
    if verbose:
        name_column_width = 20
        num_column_width  = 6
        print(f"{C.tag.ljust(name_column_width)}",'\t \t', f"{str(round(C.mass_properties.mass,2)).ljust(num_column_width)}", '\t',  global_cg_loc     )
    total_mass   += C.mass_properties.mass                 
    total_moment += C.mass_properties.mass*global_cg_loc 
    if segment != None:
        ones_row  = segment.state.ones_row  
        segment.state.conditions.weights.components.mass[C.tag]                            = C.mass_properties.mass  * ones_row(1)   
        segment.state.conditions.weights.components.global_center_of_gravity[C.tag]        = global_cg_loc * ones_row(1)  
    return total_mass,total_moment
