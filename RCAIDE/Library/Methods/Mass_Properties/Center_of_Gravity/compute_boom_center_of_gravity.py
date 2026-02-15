# RCAIDE/Library/Methods/Mass_Properties/Center_of_Gravity/compute_boom_center_of_gravity.py 
# 
# Created:  Dec 2025, M. Clarke 


# ----------------------------------------------------------------------------------------------------------------------
#  Compute Boom Center of Gravity
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_boom_center_of_gravity(boom):
    
    if boom.mass_properties.center_of_gravity[0][0] == 0: 
        boom.mass_properties.center_of_gravity[0][0] = .5*boom.lengths.total  
    
    return boom.mass_properties.center_of_gravity