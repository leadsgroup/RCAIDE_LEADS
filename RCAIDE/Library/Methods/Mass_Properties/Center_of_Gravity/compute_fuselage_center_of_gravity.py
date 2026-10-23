# RCAIDE/Library/Methods/Mass_Properties/Center_of_Gravity/compute_fuselage_center_of_gravity.py 
# 
# Created:  Dec 2025, M. Clarke 


# ----------------------------------------------------------------------------------------------------------------------
#  Compute Fuselage Center of Gravity
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_fuselage_center_of_gravity(fuselage):
    """Places an undefined fuselage CG at 45 percent of its length, the midpoint of the 40-50 percent range in
    Chai, Crisafulli, and Mason, AIAA Paper 95-3882, Table 4."""
    if fuselage.mass_properties.center_of_gravity[0][0] == 0: 
        fuselage.mass_properties.center_of_gravity[0][0] = 0.45*fuselage.lengths.total 
    
    return fuselage.mass_properties.center_of_gravity