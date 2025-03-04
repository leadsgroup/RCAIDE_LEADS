# RCAIDE/Library/Methods/Powertrain/Systems/compute_payload_power_draw.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
# package imports  
def compute_payload_power_draw(payload,payload_conditions,conditions): 
    payload_conditions.power[:,0] = payload.power_draw  
    return 