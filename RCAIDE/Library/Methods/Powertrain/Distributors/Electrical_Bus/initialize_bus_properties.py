#  RCAIDE/Methods/Energy/Distributors/Electrical_Bus/initialize_bus_properties.py
# 
# Created:  Sep 2024, S. Shekar
# Modified: Jan 2025, M. Clarke
#

import RCAIDE
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common          import compute_module_properties 
from RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Common      import compute_stack_properties

# ----------------------------------------------------------------------------------------------------------------------
#  METHODS
# ---------------------------------------------------------------------------------------------------------------------- 
def initialize_bus_properties(bus,network): 
    """
    Initializes the bus electrical properties based on what is appended onto the bus.
    
    Parameters
    ----------
    bus : ElectricalBus
        The electrical bus component with the following attributes:
            - battery_modules : list
                List of battery modules connected to the bus
            - battery_module_electric_configuration : str
                Configuration of battery modules ('Series' or 'Parallel')
            - fuel_cell_stacks : list
                List of fuel cell stacks connected to the bus
            - fuel_cell_stack_electric_configuration : str
                Configuration of fuel cell stacks ('Series' or 'Parallel')
    
    Returns
    -------
    None
        This function modifies the bus object in-place, setting the following attributes:
            - voltage : float
                Bus voltage [V]
            - nominal_capacity : float
                Nominal capacity [Ah]
            - maximum_energy : float
                Maximum energy storage capacity [J]
    
    Notes
    -----
    This function calculates the electrical properties of the bus based on the connected
    energy sources (battery modules and fuel cell stacks). It handles both series and 
    parallel configurations.
    
    For battery modules:
        - In series configuration: voltages add, capacity is the maximum of all modules
        - In parallel configuration: voltage is the maximum voltage of all modules, capacities add
    
    For fuel cell stacks:
        - In series configuration: voltages add
        - In parallel configuration: voltage is the maximum voltage of all stacks
    
    The function first computes properties for each individual module/stack by calling
    their respective property computation functions.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common.compute_module_properties
    RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Common.compute_stack_properties
    """
    # loops through the sources, if electrical, update the bus that it is on
    number_of_sources = 0
    cumulative_fuel_cell_stack = 0
    power_split_ratio = 0
    for source in network.sources:
        if source.active and (bus.tag in source.assigned_distributors[0]): 
            number_of_sources += 1
            
            # compute power split ratio
            power_split_ratio = 1/number_of_sources 
            
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Generic_Battery_Module): 
                bus            = network.distributors[source.assigned_distributors[0][0]] 
                if bus.battery_module_electric_configuration == 'Series':
                    compute_module_properties(source) 
                    bus.voltage         +=  source.voltage
                elif bus.battery_module_electric_configuration == 'Parallel': 
                    compute_module_properties(source)        
                    bus.voltage           =  max(source.voltage, bus.voltage)
            elif isinstance(source, RCAIDE.Library.Components.Powertrain.Converters.Generic_Fuel_Cell_Stack): 
                fuel_cell_stack =  source 
                bus             = network.distributors[source.assigned_distributors[0][0]] 
                if bus.fuel_cell_stack_electric_configuration == 'Series':  
                    compute_stack_properties(fuel_cell_stack)
                    cumulative_fuel_cell_stack += fuel_cell_stack.voltage 
                    bus.voltage     =  min(fuel_cell_stack.voltage, cumulative_fuel_cell_stack) 
                elif bus.fuel_cell_stack_electric_configuration == 'Parallel':  
                    compute_stack_properties(fuel_cell_stack)        
                    bus.voltage     =  max(fuel_cell_stack.voltage, bus.voltage)
                    
                
    for source in network.sources:
        if source.active and (bus.tag in source.assigned_distributors[0]):
            source.power_split_ratio = power_split_ratio
    return