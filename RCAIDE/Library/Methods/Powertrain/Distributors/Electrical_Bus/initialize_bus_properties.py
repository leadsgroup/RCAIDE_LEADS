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
    """ Initializes the bus electrical properties based what is appended onto the bus
        
        Assumptions:
        N/A
    
        Source:
        N/A
    
        Inputs:  
       
        Outputs:
           
        Properties Used:
        None
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