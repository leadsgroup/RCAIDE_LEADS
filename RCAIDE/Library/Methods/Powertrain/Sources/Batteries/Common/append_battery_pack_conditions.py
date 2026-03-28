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
def append_battery_pack_conditions(battery,segment): 
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
      
    segment.state.conditions.energy.sources[battery.tag]                                     = Conditions()  
    segment.state.conditions.energy.sources[battery.tag].power_draw                          = 0 * ones_row(1) 
    segment.state.conditions.energy.sources[battery.tag].state_of_charge                     = 0 * ones_row(1) 
    segment.state.conditions.energy.sources[battery.tag].depth_of_discharge                  = 0 * ones_row(1) 
    segment.state.conditions.energy.sources[battery.tag].current_draw                        = 0 * ones_row(1)
    segment.state.conditions.energy.sources[battery.tag].charging_current                    = 0 * ones_row(1)
    segment.state.conditions.energy.sources[battery.tag].voltage_open_circuit                = 0 * ones_row(1)
    segment.state.conditions.energy.sources[battery.tag].voltage_under_load                  = 0 * ones_row(1) 
    segment.state.conditions.energy.sources[battery.tag].heat_energy_generated               = 0 * ones_row(1) 
    segment.state.conditions.energy.sources[battery.tag].efficiency                          = 0 * ones_row(1)
    segment.state.conditions.energy.sources[battery.tag].temperature                         = 0 * ones_row(1)
    segment.state.conditions.energy.sources[battery.tag].energy                              = 0 * ones_row(1)
    segment.state.conditions.energy.sources[battery.tag].regenerative_power                  = 0 * ones_row(1)    
    segment.state.conditions.energy.sources[battery.tag].power_split_ratio                   = ones_row(1)       # NEEED TO UPDATE battery.power_split_ratio * ones_row(1)
    segment.state.conditions.energy.sources[battery.tag].inputs                              = Conditions()
    segment.state.conditions.energy.sources[battery.tag].inputs.power                        = Conditions()  
    segment.state.conditions.energy.sources[battery.tag].inputs.power.electrical             = 0 * ones_row(1)
    segment.state.conditions.energy.sources[battery.tag].inputs.power.thermal                = 0 * ones_row(1) 
    segment.state.conditions.energy.sources[battery.tag].inputs.power.hydraulic              = 0 * ones_row(1)   
    segment.state.conditions.energy.sources[battery.tag].outputs                             = Conditions()  
    segment.state.conditions.energy.sources[battery.tag].outputs.power                       = Conditions()  
    segment.state.conditions.energy.sources[battery.tag].outputs.power.electrical            = 0 * ones_row(1) 
    segment.state.conditions.energy.sources[battery.tag].outputs.power.thermal               = 0 * ones_row(1) 
    segment.state.conditions.energy.sources[battery.tag].outputs.power.hydraulic             = 0 * ones_row(1)       
     
    for module in battery.modules: 
        module.append_operating_conditions(battery,segment)
    return 
    
def append_battery_pack_segment_conditions(battery, segment): 
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
    battery_conditions.power_draw[:,0]            = 0 
    battery_conditions.inputs.power.electrical[:,0] = 0
    battery_conditions.outputs.power.electrical[:,0] = 0
    battery_conditions.temperature[:,0] = 0
    battery_conditions.energy[:,0] = 0
    battery_conditions.state_of_charge[:,0] = 0
    battery_conditions.heat_energy_generated[:,0] = 0
    battery_conditions.voltage_open_circuit[:,0] = 0
    battery_conditions.voltage_under_load[:,0] = 0
    
    # Thermal power draw
    if segment.state.initials:
        for network in segment.analyses.vehicle.networks:
            for distributor in  network.distributors:
                pass
                #for tag, item in  coolant_line.items():
                    #if tag == 'battery_modules':
                        #for battery in item:
                            #for btms in  battery:
                                #battery_conditions.power_draw[0,0]   +=  segment.state.initials.conditions.energy.coolant_lines[coolant_line.tag][btms.tag].power[-1] 
                    #if tag == 'heat_exchangers':
                        #for heat_exchanger in  item:                    
                            #battery_conditions.power_draw[0,0]   +=  segment.state.initials.conditions.energy.coolant_lines[coolant_line.tag][heat_exchanger.tag].power[-1] 
        # Bus Properties 
        battery_initials            = segment.state.initials.conditions.energy.sources[battery.tag]  
        if type(segment) ==  RCAIDE.Framework.Mission.Segments.Ground.Battery_Recharge:             
            battery_initials.battery_discharge_flag           = False 
        else:                   
            battery_initials.battery_discharge_flag           = True     
        battery_conditions.energy[0,0]          = battery_initials.energy[-1,0] 
        battery_conditions.temperature[0,0]     = battery_initials.temperature[-1,0]


    return    