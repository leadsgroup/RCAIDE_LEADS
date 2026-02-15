# RCAIDE/Library/Methods/Mass_Properties/Center_of_Gravity/compute_cylinder_center_of_gravity.py 
# 
# Created:  Dec 2025, M. Clarke 


# ----------------------------------------------------------------------------------------------------------------------
#  Compute Cylinder Center of Gravity
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_cylinder_center_of_gravity(component, length = 0):
    
    if component.mass_properties.center_of_gravity[0][0] == 0: 
        component.mass_properties.center_of_gravity[0][0] = .5*length
    
    return component.mass_properties.center_of_gravity