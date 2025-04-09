#  RCAIDE/Methods/Energy/Distributors/Electrical_Bus/initialize_bus_properties.py
# 
# Created:  Sep 2024, S. Shekar
# Modified: Jan 2025, M. Clarke
#
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common          import compute_module_properties 
from RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Common             import compute_stack_properties

# ----------------------------------------------------------------------------------------------------------------------
#  METHODS
# ---------------------------------------------------------------------------------------------------------------------- 
def initialize_bus_properties(bus): 
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
    if len(bus.battery_modules) > 0: 
        if bus.battery_module_electric_configuration == 'Series':
            bus.nominal_capacity = 0
            bus.maximum_energy   = 0
            for battery_module in  bus.battery_modules: 
                compute_module_properties(battery_module) 
                bus.voltage         +=  battery_module.voltage
                bus.maximum_energy  +=  battery_module.maximum_energy
                bus.nominal_capacity =  max(battery_module.nominal_capacity, bus.nominal_capacity)  
        elif bus.battery_module_electric_configuration == 'Parallel':
            bus.voltage = 0
            bus.maximum_energy   = 0
            for battery_module in  bus.battery_modules: 
                compute_module_properties(battery_module)        
                bus.voltage           =  max(battery_module.voltage, bus.voltage)
                bus.nominal_capacity +=  battery_module.nominal_capacity        
                bus.maximum_energy  +=  battery_module.initial_maximum_energy
    
    if len(bus.fuel_cell_stacks) > 0: 
        if bus.fuel_cell_stack_electric_configuration == 'Series':
            bus.maximum_energy   = 0
            for fuel_cell_stack in  bus.fuel_cell_stacks: 
                compute_stack_properties(fuel_cell_stack) 
                bus.voltage         +=  fuel_cell_stack.voltage  
        elif bus.fuel_cell_stack_electric_configuration == 'Parallel': 
            for fuel_cell_stack in  bus.fuel_cell_stacks: 
                compute_stack_properties(fuel_cell_stack)        
                bus.voltage           =  max(fuel_cell_stack.voltage, bus.voltage)              
    return