# RCAIDE/Library/Methods/Powertrain/Systems/compute_avionics_performance.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
# package imports
def compute_avionics_power_draw(avionics,avionics_conditions,conditions): 
    avionics_conditions.power[:,0] = avionics.power_draw  
    return 