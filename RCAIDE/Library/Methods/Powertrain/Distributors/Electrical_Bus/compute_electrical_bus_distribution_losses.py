# RCAIDE/Methods/Powertrain/Distributors/Electrical_Bus/compute_electrical_bus_distribution_losses.py
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
# compute_electrical_bus_distribution_losses
# ----------------------------------------------------------------------------------------------------------------------
def compute_electrical_bus_distribution_losses(bus,component_conditions,state,network): 
    
    Power   = abs(component_conditions.outputs.power.electrical - component_conditions.inputs.power.electrical)
    P_cable = Power/ bus.number_of_parallel_wires 
    bus_conditions = state.conditions.energy.distributors[bus.tag]   
    
    # 1. The instantaneous power demand for this specific time step 
    Voltage    = bus_conditions.voltage
    resistance = bus.conductor.resistance 

    # 2. Loss calculation for the whole time array 
    discriminant = (-Voltage)**2 - 4*resistance*P_cable
        
    I_mission_array = (Voltage - np.sqrt(discriminant)) / (2*resistance)
    P_loss = (I_mission_array**2) * resistance 
    
    # 3. Write results back to the state arrays
    bus_conditions.inputs.power.electrical += P_loss   
    bus_conditions.outputs.power.thermal   += P_loss

    return  