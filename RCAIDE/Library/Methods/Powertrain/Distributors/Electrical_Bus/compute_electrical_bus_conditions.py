# RCAIDE/Methods/Powertrain/Distributors/Electrical_Bus/compute_electrical_bus_conditions.py
# 
# 
# Created: Mar 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# imports 
import numpy as np
 
# ----------------------------------------------------------------------------------------------------------------------
# compute_bus_conditions
# ----------------------------------------------------------------------------------------------------------------------
def compute_electrical_bus_conditions(bus, state,network): 
    bus_conditions              = state.conditions.energy.distributors[bus.tag]       
    return bus_conditions.inputs, bus_conditions.outputs