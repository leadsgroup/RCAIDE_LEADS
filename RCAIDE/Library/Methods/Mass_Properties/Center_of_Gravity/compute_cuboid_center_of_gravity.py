# RCAIDE/Library/Methods/Mass_Properties/Center_of_Gravity/compute_cuboid_center_of_gravity.py 
# 
# Created:  Dec 2025, M. Clarke 


# ----------------------------------------------------------------------------------------------------------------------
#  Compute Cuboid Center of Gravity
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_cuboid_center_of_gravity(component, length=0):
    """
    Compute the center of gravity of a cuboid
    
    Assumptions:
     - component is position along its axis of symmetric in the y and z axis
     - uniform density of component
     - center of gravity along the x axis is loccated and length/2
     """
    
    if component.mass_properties.center_of_gravity[0][0] == 0: 
        component.mass_properties.center_of_gravity[0][0] = .5*length    
    
    return component.mass_properties.center_of_gravity