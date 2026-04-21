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
    
    size_electrical_cable(bus)
    
    return 