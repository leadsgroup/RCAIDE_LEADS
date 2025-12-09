# RCAIDE/Methods/Stability/Common/mass_and_intertia_functions.py 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports
import RCAIDE
from RCAIDE.Library.Components         import Component  

# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Recursive Moment
# ----------------------------------------------------------------------------------------------------------------------   
def sum_moment(component, total_mass, total_moment):
    """ Recursively sums up the moment of all Components and subcomponents

    Assumptions:
    None

    Source:
    N/A

    Inputs:
       compoment

    Outputs:
       total_moment
       total_mass
    """    
    for key,Comp in component.items():
        if  isinstance(Comp,Component.Container):
            total_moment , total_mass  = sum_moment(Comp, total_mass, total_moment) 
            
        elif isinstance(Comp,Component):  
            total_mass, total_moment = update_mass_and_moment(total_mass,total_moment,Comp) 
        
            for key in Comp.keys():
                item = Comp[key]
                if isinstance(item,Component.Container):
                    total_moment , total_mass  = sum_moment(item, total_mass, total_moment)
                if isinstance(item,Component): 
                    total_mass, total_moment = update_mass_and_moment(total_mass,total_moment,item) 
            
    return total_moment , total_mass

def update_mass_and_moment(total_mass,total_moment,C): 
    global_cg_loc = np.array(C.mass_properties.center_of_gravity) + np.array(C.origin) 
    if isinstance(C,RCAIDE.Library.Components.Landing_Gear.Landing_Gear) or isinstance(C,RCAIDE.Library.Components.Wings.Wing):
        if C.xz_plane_symmetric:
            global_cg_loc[0][1] = 0
    if global_cg_loc[0][0] == 0:
        pass
    else:    
        M = C.mass_properties.mass 
        if M != 0:
            total_mass   += M                 
            total_moment += M*global_cg_loc 
    
    return total_mass,total_moment

