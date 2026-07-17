# RCAIDE/Library/Methods/Mass_Properties/Center_of_Gravity/compute_wing_integral_tank_center_of_gravity.py 
# 
# Created:  Dec 2025, M. Clarke 
 
# ----------------------------------------------------------------------------------------------------------------------
#  Compute Boom Center of Gravity
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_wing_integral_tank_center_of_gravity(fuel_tank,vehicle):    
    fuel_tank.mass_properties.center_of_gravity = fuel_tank.fuel.mass_properties.center_of_gravity  
    return  fuel_tank.mass_properties.center_of_gravity 