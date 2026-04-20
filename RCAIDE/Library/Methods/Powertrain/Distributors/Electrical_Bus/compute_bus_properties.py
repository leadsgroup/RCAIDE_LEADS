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

import numpy as np
# ----------------------------------------------------------------------------------------------------------------------
#  compute_bus_properties
# ----------------------------------------------------------------------------------------------------------------------
def compute_bus_properties(bus,network):

    # for propulsor in network.propulsors:
    #     if bus.tag in propulsor.assigned_distributors[0]:
    #         bus.design_voltage = np.maximum(bus.design_voltage,propulsor.design_voltage)
    #         bus.design_power   = np.maximum(bus.design_power,propulsor.design_power)     
    
    # for converter in network.converters:
    #     if bus.tag in converter.assigned_distributors[0]:
    #         bus.design_voltage = np.maximum(bus.design_voltage,converter.design_voltage)
    #         bus.design_power   = np.maximum(bus.design_power,converter.design_power) 
        
    # for source in network.sources:
    #     if bus.tag in source.assigned_distributors[0]: 
    #         bus.design_voltage = np.maximum(bus.design_voltage,source.voltage)
    #         bus.design_power   = np.maximum(bus.design_power,source.design_power)
    
    size_electrical_cable(bus)
    
    return 