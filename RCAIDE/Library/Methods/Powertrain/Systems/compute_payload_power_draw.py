# RCAIDE/Library/Methods/Powertrain/Systems/compute_payload_power_draw.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
# package imports  
def compute_payload_power_draw(payload,bus,conditions): 
    bus_conditions                = conditions.energy[bus.tag]
    payload_conditions            = bus_conditions[payload.tag]    
    payload_conditions.power[:,0] = payload.power_draw  
    bus_conditions.power_draw     += payload_conditions.power*bus.power_split_ratio /bus.efficiency
    return 