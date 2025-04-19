# RCAIDE/Methods/Powertrain/Sources/Batteries/Aluminum_Air/compute_al_air_cell_performance.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  compute_al_air_cell_performance
# ----------------------------------------------------------------------------------------------------------------------  
def find_aluminum_mass(battery, energy):
    aluminum_mass = energy*battery.cell.aluminum_mass_factor
    return aluminum_mass 

def find_water_mass(battery, energy):
    water_mass = energy*battery.cell.water_mass_gain_factor
    return water_mass
