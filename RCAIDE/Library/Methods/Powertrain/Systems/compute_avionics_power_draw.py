# RCAIDE/Library/Methods/Powertrain/Systems/compute_avionics_performance.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
# package imports
def compute_avionics_power_draw(avionics,bus,conditions):
    bus_conditions                 = conditions.energy[bus.tag]
    avionics_conditions            = bus_conditions[avionics.tag]    
    avionics_conditions.power[:,0] = avionics.power_draw 
    bus_conditions.power_draw      += avionics_conditions.power*bus.power_split_ratio /bus.efficiency    
    return 