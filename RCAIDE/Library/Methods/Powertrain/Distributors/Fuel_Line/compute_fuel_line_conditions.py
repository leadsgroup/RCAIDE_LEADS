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
def compute_fuel_line_conditions(fuel_line, state,network): 
    fuel_line_conditions              = state.conditions.energy.distributors[fuel_line.tag]       
    return fuel_line_conditions.inputs, fuel_line_conditions.outputs