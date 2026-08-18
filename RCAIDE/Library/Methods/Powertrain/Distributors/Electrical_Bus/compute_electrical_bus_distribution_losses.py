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
    """
    Computes the resistive line losses of an electrical bus's cable and accumulates
    the resulting power draw and waste heat onto the bus's conditions.

    Parameters
    ----------
    bus : RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus
        Electrical bus whose cable losses are being computed, with the following
        attributes:
            - number_of_parallel_wires : int
                Number of parallel wires sharing the bus's total power
            - conductor.resistance : float
                Fixed total electrical resistance of the sized cable [Ohm], from
                size_electrical_cable
    component_conditions : Conditions
        Conditions of the source, converter, propulsor, or system assigned to this
        bus for which this call is computing a power contribution (this function is
        called once per assigned component, so it runs multiple times per bus per
        iteration)
    state : RCAIDE.Framework.Mission.Common.State
        Mission segment state
    network : RCAIDE.Framework.Networks.Network
        The network this bus belongs to

    Returns
    -------
    None

    Notes
    -----
    The current draw is found from the positive root of the quadratic
    R*I^2 - V*I + P_cable = 0, where P_cable is this component's share of the bus's
    total power split evenly across number_of_parallel_wires. The resulting I^2*R
    loss is added to both the bus's electrical power input (it must be supplied in
    addition to the component's own demand) and its thermal power output (as waste
    heat).

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus.compute_distribution_losses
    RCAIDE.Library.Methods.Powertrain.Distributors.Electrical_Bus.size_electrical_cable
    """
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