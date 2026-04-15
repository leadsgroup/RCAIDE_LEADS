# RCAIDE/Methods/Powertrain/Distributors/Electrical_Bus/compute_bus_properties.py
# 
# 
# Created: Mar 2026, M. Clarke
# Modified: Apr 2026, S. Sharma

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# imports  
import RCAIDE 
from .size_electrical_cable import size_electrical_cable 

# ----------------------------------------------------------------------------------------------------------------------
#  compute_bus_properties
# ----------------------------------------------------------------------------------------------------------------------
def compute_bus_properties(bus,network):
    
    for source in network.sources:
        if bus.tag in source.assigned_distributors[0]: 
            bus.design_voltage += source.voltage
    
    size_electrical_cable(bus)
    
    return 