# RCAIDE/Methods/Powertrain/Distributors/Electrical_Bus/compute_bus_properties.py
# 
# 
# Created: Mar 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_bus_properties
# ----------------------------------------------------------------------------------------------------------------------
def compute_bus_properties(bus,network):
    
    for source in network.sources:
        if bus.tag in source.assigned_distributors[0]: 
            bus.voltage += source.voltage
    
    return 