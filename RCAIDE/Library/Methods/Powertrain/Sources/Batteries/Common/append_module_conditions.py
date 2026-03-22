# RCAIDE/Methods/Powertrain/Sources/Batteries/Common/append_battery_module_conditions.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Sep 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE 
from RCAIDE.Framework.Mission.Common     import   Conditions

# ----------------------------------------------------------------------------------------------------------------------
#  METHODS
# ----------------------------------------------------------------------------------------------------------------------  
def append_module_conditions(module,battery,segment): 
    """ Appends the initial battery conditions
    
        Assumptions:
        ------------
        Battery temperature is set to one degree hotter than ambient 
        temperature for robust convergence. Initial mission energy, maxed aged energy, and 
        initial segment energy are the same. Cycle day is zero unless specified, resistance_growth_factor and
        capacity_fade_factor is one unless specified in the segment
    
        Source:
        N/A
    
        Inputs:  
            atmosphere.temperature             [Kelvin]
            
            Optional:
            segment.
                 battery_module.cycle_in_day               [unitless]
                 battery_module.module.temperature         [Kelvin]
                 battery_module.charge_throughput          [Ampere-Hours] 
                 battery_module.resistance_growth_factor   [unitless]
                 battery_module.capacity_fade_factor       [unitless]
                 battery_module.discharge                  [boolean]
                 increment_battery_age_by_one_day   [boolean]
               
        Outputs:
            segment
               battery_discharge                    [boolean]
               increment_battery_age_by_one_day     [boolean]
               segment.state.conditions.energy
               battery_module.battery_discharge_flag         [boolean]
               battery_module.maximum_initial_energy  [watts]
               battery_module.energy                  [watts] 
               battery_module.temperature             [kelvin]
               battery_module.cycle_in_day                   [int]
               battery_module.cell.charge_throughput         [Ampere-Hours] 
               battery_module.resistance_growth_factor       [unitless]
               battery_module.capacity_fade_factor           [unitless] 
    
        Properties Used:
        None
    """
 
    ones_row  = segment.state.ones_row
    
    segment.state.conditions.energy.sources[battery.tag][module.tag] = Conditions() 
    module_conditions                                 = segment.state.conditions.energy.sources[battery.tag][module.tag] 

    module_conditions.state_of_charge                 = 0 * ones_row(1)
    module_conditions.temperature                     = 0 * ones_row(1)
    module_conditions.energy                          = 0 * ones_row(1) 
    module_conditions.voltage_open_circuit            = 0 * ones_row(1)
    module_conditions.voltage_under_load              = 0 * ones_row(1)
    module_conditions.internal_resistance             = 0 * ones_row(1)
    module_conditions.heat_energy_generated           = 0 * ones_row(1)
    module_conditions.current                         = 0 * ones_row(1)
    module_conditions.power                           = 0 * ones_row(1)
    
    module_conditions.inputs                          = Conditions()
    module_conditions.inputs.power                    = Conditions()  
    module_conditions.inputs.power.electrical         = 0 * ones_row(1)
    
    module_conditions.outputs                         = Conditions()  
    module_conditions.outputs.power                   = Conditions()  
    module_conditions.outputs.power.electrical        = 0 * ones_row(1)    
    
    module_conditions.cell                            = Conditions() 
    module_conditions.cell.voltage_open_circuit       = 0 * ones_row(1)
    module_conditions.cell.internal_resistance        = 0 * ones_row(1)
    module_conditions.cell.voltage_under_load         = 0 * ones_row(1)
    module_conditions.cell.power                      = 0 * ones_row(1) 
    module_conditions.cell.current                    = 0 * ones_row(1)   
    module_conditions.cell.heat_energy_generated      = 0 * ones_row(1)   
    module_conditions.cell.energy                     = 0 * ones_row(1)
    module_conditions.cell.cycle_in_day               = 0
    module_conditions.cell.resistance_growth_factor   = 1.
    module_conditions.cell.capacity_fade_factor       = 1.  
        
    # first segment 
    if 'initial_battery_state_of_charge' in segment:
    
        n_series          = module.electrical_configuration.series
        n_parallel        = module.electrical_configuration.parallel 
        n_total           = n_series*n_parallel
        
        initial_battery_energy                          = segment.initial_battery_state_of_charge*battery.maximum_energy   
        module_conditions.cell.maximum_initial_energy   = initial_battery_energy
        module_conditions.cell.energy                   = initial_battery_energy* ones_row(1) 
        module_conditions.cell.energy                   = initial_battery_energy / n_total* ones_row(1) 
        module_conditions.cell.state_of_charge          = segment.initial_battery_state_of_charge* ones_row(1) 
        module_conditions.cell.state_of_charge          = segment.initial_battery_state_of_charge* ones_row(1) 
        module_conditions.cell.depth_of_discharge       = 1 - segment.initial_battery_state_of_charge* ones_row(1)
    else:  
        module_conditions.cell.energy               = 0 * ones_row(1)
        module_conditions.cell.state_of_charge      = 0 * ones_row(1)
        module_conditions.cell.state_of_charge      = 0 * ones_row(1)       
        module_conditions.cell.depth_of_discharge   = 0 * ones_row(1) 
        
    # temperature 
    if 'battery_cell_temperature' in segment:
        cell_temperature  = segment.battery_cell_temperature  
    else: 
        # compute ambient conditions
        atmosphere    = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
        alt           = -segment.conditions.frames.inertial.position_vector[:,2] 
        if segment.temperature_deviation != None:
            temp_dev = segment.temperature_deviation    
        atmo_data    = atmosphere.compute_values(altitude = alt,temperature_deviation=temp_dev)        
        cell_temperature                                 = atmo_data.temperature[0,0] 
    module_conditions.cell.temperature      = cell_temperature * ones_row(1)         
    module_conditions.cell.temperature      = cell_temperature * ones_row(1) 

    # charge thoughput 
    if 'charge_throughput' in segment: 
        module_conditions.cell.charge_throughput          = segment.charge_throughput * ones_row(1)  
        module_conditions.cell.resistance_growth_factor   = segment.resistance_growth
        module_conditions.cell.capacity_fade_factor       = segment.capacity_fade
        module_conditions.cell.cycle_in_day               = segment.cycle_day 
        module_conditions.cell.resistance_growth_factor   = 1 
        module_conditions.cell.capacity_fade_factor       = 1 
        module_conditions.cell.cycle_in_day               = 0
    else:
        module_conditions.cell.charge_throughput    = 0 * ones_row(1)     
     
    return 
    
def append_battery_module_segment_conditions(module,battery, segment): 
    """Sets the initial battery energy at the start of each segment as the last point from the previous segment 
    
        Assumptions:
        None
    
        Source:
        N/A
    
        Inputs:  
         battery          (data structure)              [None]
               
        Outputs:
        None
    
        Properties Used:
        None
    """ 
    battery_conditions = segment.state.conditions.energy.sources[battery.tag]

    battery_conditions.inputs.power.electrical[:,0]    = 0.0   
    battery_conditions.outputs.power.electrical[:,0]   = 0.0      
    module_conditions  = battery_conditions[module.tag]
    
    if segment.state.initials:         
        module_initials    = segment.state.initials.conditions.energy.sources[battery.tag][module.tag]
        module_conditions.cell.temperature[:,0]           = module_initials.cell.temperature[-1,0]
        module_conditions.cell.cycle_in_day               = module_initials.cell.cycle_in_day      
        module_conditions.cell.charge_throughput[:,0]     = module_initials.cell.charge_throughput[-1,0]
        module_conditions.cell.resistance_growth_factor   = module_initials.cell.resistance_growth_factor 
        module_conditions.cell.capacity_fade_factor       = module_initials.cell.capacity_fade_factor 
        module_conditions.cell.state_of_charge[:,0]       = module_initials.cell.state_of_charge[-1,0]
        module_conditions.cell.energy[:,0]                = module_initials.cell.energy[-1,0]  
        
        #battery_conditions.energy[:,0]                     = battery_initials.energy[-1,0]
        module_conditions.temperature[:,0]                = module_initials.temperature[-1,0]
            

    if 'battery_cell_temperature' in segment:        
        module_conditions.cell.temperature[:,0]           = segment.battery_cell_temperature
        module_conditions.temperature[:,0]               = segment.battery_cell_temperature       
       
    if 'initial_battery_state_of_charge' in segment:    
        n_series                                          = module.electrical_configuration.series
        n_parallel                                        = module.electrical_configuration.parallel 
        n_total                                           = n_series*n_parallel 
        module_conditions.cell.energy[:,0]                = segment.initial_battery_state_of_charge*battery.maximum_energy / n_total 
        module_conditions.cell.state_of_charge[:,0]       = segment.initial_battery_state_of_charge
        module_conditions.cell.depth_of_discharge[:,0]    = 1 - segment.initial_battery_state_of_charge

    return    