# RCAIDE/Library/Methods/Mass_Properties/Center_of_Gravity/compute_cargo_bay_center_of_gravity.py 
# 
# Created:  Dec 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Cargo Bay Center of Gravity
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_cargo_bay_center_of_gravity(cargo_bay):
    """Compute the center of gravity of a cargo bay
    Assumtions
    ___________
    Cargo bay is a cubiod 
    """ 
    cargo_bay.mass_properties.center_of_gravity           = [[cargo_bay.length / 2,0,0 ]]     
    
    return cargo_bay.mass_properties.center_of_gravity