# RCAIDE/Library/Methods/Mass_Properties/Center_of_Gravity/compute_fuselage_center_of_gravity.py 
# 
# Created:  Dec 2025, M. Clarke 


# ----------------------------------------------------------------------------------------------------------------------
#  Compute Fuselage Center of Gravity
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_fuselage_center_of_gravity(fuselage):
    
    if fuselage.mass_properties.center_of_gravity[0][0] == 0: 
        fuselage.mass_properties.center_of_gravity[0][0] = .51*fuselage.lengths.total 
    
    return fuselage.mass_properties.center_of_gravity