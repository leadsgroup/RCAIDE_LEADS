# RCAIDE/Methods/Powertrain/Sources/Batteries/Lithium_Ion_NMC/update_SLPB_cell_age.py
# 
# Created:  May 2025, C. Boggan 
 
# ----------------------------------------------------------------------------------------------------------------------
# update_SLPB_cell_age
# ----------------------------------------------------------------------------------------------------------------------  
def update_SLPB_cell_age(battery_conditions, increment_battery_age_by_one_day):  

    if increment_battery_age_by_one_day: 
        battery_conditions.cell.cycle_in_day += 1 
  
    return  
