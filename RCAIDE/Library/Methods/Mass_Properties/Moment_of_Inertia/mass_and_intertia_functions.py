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
    
    
# ----------------------------------------------------------------------------------------------------------------------
#  Recursive Moment of Intertia 
# ----------------------------------------------------------------------------------------------------------------------   
def sum_moment_of_inertia(component, vehicle_center_of_gravity = None): 
    """ Recursively sums up the moment of intertia of all Components and subcomponents

    Assumptions:
    None

    Source:
    N/A

    Inputs:
       compoment
       vehicle_center_of_gravity

    Outputs:
       total_I 
    """   
    total_I = np.array([[0.0,0.0,0.0]]) 
    for key,Comp in component.items():
        if  isinstance(Comp,Component.Container):
            total_I += sum_moment_of_inertia(Comp,vehicle_center_of_gravity )   
        elif isinstance(Comp,Component):   
            global_cg_loc = Comp.mass_properties.center_of_gravity + Comp.origin 
            total_I += Comp.mass_properties.moments_of_inertia.center + Comp.mass_properties.mass*((vehicle_center_of_gravity - global_cg_loc)**2)
            
            for key in Comp.keys():
                item = Comp[key]
                if isinstance(item,Component.Container):
                    total_I += sum_moment_of_inertia(Comp,vehicle_center_of_gravity )             
    return total_I
 

