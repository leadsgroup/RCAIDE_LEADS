# RCAIDE/Methods/Powertrain/Distributors/Electrical_Bus/compute_electrical_bus_conditions.py
# 
# 
# Created: Mar 2026, M. Clarke
# Modified: Apr 2026, S. Sharma

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
    
    # 1. The instantaneous power demand for this specific time step
    Power = bus_conditions.outputs.power.electrical # CHECK THAT Matthew
    Voltage = bus_conditions.voltage
    resistance = bus.conductor.resistance 
    
    # 2. Loss calculation for the whole time array
    a = resistance
    b = -Voltage
    c = Power
    
    discriminant = b**2 - 4*a*c
        
    I_mission_array = (-b - np.sqrt(discriminant)) / (2*a)
    P_loss = (I_mission_array**2) * resistance
    
    # 3. Write results back to the state arrays
    bus_conditions.inputs.power.electrical = Power + P_loss # CHECK THAT Matthew
    
    # Store the heat loss to track thermal loads
    bus_conditions.inputs.power.thermal = P_loss
        
    return bus_conditions.inputs, bus_conditions.outputs