#  RCAIDE/Methods/Energy/Distributors/Electrical_Bus/append_battery_conditions.py
# 
# Created: Sep 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
from RCAIDE.Framework.Mission.Common     import   Conditions

# ----------------------------------------------------------------------------------------------------------------------
#  METHODS
# ---------------------------------------------------------------------------------------------------------------------- 
def append_bus_conditions(bus,segment): 
    """ Appends the initial bus conditions
        
        Assumptions:
        N/A
    
        Source:
        N/A
    
        Inputs:  
       
        Outputs:
           
        Properties Used:
        None
        """
    ones_row                                                       = segment.state.ones_row
   
    segment.state.conditions.energy[bus.tag]                       = Conditions()
    segment.state.conditions.energy[bus.tag].battery_modules       = Conditions()
    segment.state.conditions.energy[bus.tag].power_draw            = 0 * ones_row(1)
    segment.state.conditions.energy[bus.tag].SOC                   = 0 * ones_row(1) 
    segment.state.conditions.energy[bus.tag].current_draw          = 0 * ones_row(1)
    segment.state.conditions.energy[bus.tag].charging_current      = 0 * ones_row(1)
    segment.state.conditions.energy[bus.tag].voltage_open_circuit  = 0 * ones_row(1)
    segment.state.conditions.energy[bus.tag].voltage_under_load    = 0 * ones_row(1) 
    segment.state.conditions.energy[bus.tag].heat_energy_generated = 0 * ones_row(1) 
    segment.state.conditions.energy[bus.tag].efficiency            = 0 * ones_row(1)
    segment.state.conditions.energy[bus.tag].temperature           = 0 * ones_row(1)
    segment.state.conditions.energy[bus.tag].energy                = 0 * ones_row(1)
    segment.state.conditions.energy[bus.tag].regenerative_power    = 0 * ones_row(1)
    
    # TO BE INCLUDED IN NEAR FUTURE
    #if bus.battery_module_electric_configuration is 'Series':
        #bus.nominal_capacity = 0
        #for battery_module in  bus.battery_modules:
            #bus.voltage  +=   battery_module.voltages
            #bus.nominal_capacity =  battery_module.nominal_capacity               
        
    #elif bus.battery_module_electric_configuration is 'Parallel':
        #for battery_module in  bus.battery_modules:
            #bus.voltage +=  battery_module.voltage
            #bus.capacity =  battery_module.capacity        
    #else: raise Exception("Define how the battery modules are connected on the bus")
   
    return    