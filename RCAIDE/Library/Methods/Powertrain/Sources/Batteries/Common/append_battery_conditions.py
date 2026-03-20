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
def append_battery_conditions(battery,segment): 
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
    
    segment.state.conditions.energy.sources[battery.tag]                                 = Conditions()  
    segment.state.conditions.energy.sources[battery.tag].voltage_open_circuit            = 0 * ones_row(1) 
    segment.state.conditions.energy.sources[battery.tag].internal_resistance             = 0 * ones_row(1)
    segment.state.conditions.energy.sources[battery.tag].voltage_under_load              = 0 * ones_row(1)  
    segment.state.conditions.energy.sources[battery.tag].heat_energy_generated           = 0 * ones_row(1) 
    segment.state.conditions.energy.sources[battery.tag].energy                          = 0 * ones_row(1)     
    segment.state.conditions.energy.sources[battery.tag].power_draw                      = 0 * ones_row(1)    
    segment.state.conditions.energy.sources[battery.tag].current_draw                    = 0 * ones_row(1) 
    segment.state.conditions.energy.sources[battery.tag].current                         = 0 * ones_row(1) 
     
    segment.state.conditions.energy.sources[battery.tag].power_split_ratio               = battery.power_split_ratio  
    segment.state.conditions.energy.sources[battery.tag].inputs                          = Conditions()
    segment.state.conditions.energy.sources[battery.tag].inputs.power                    = Conditions()  
    segment.state.conditions.energy.sources[battery.tag].inputs.power.electrical         = 0 * ones_row(1) 
           
    segment.state.conditions.energy.sources[battery.tag].outputs                         = Conditions()  
    segment.state.conditions.energy.sources[battery.tag].outputs.power                   = Conditions()  
    segment.state.conditions.energy.sources[battery.tag].outputs.power.electrical        = 0 * ones_row(1) 
    
    # Conditions for recharging battery module
    if isinstance(segment,RCAIDE.Framework.Mission.Segments.Ground.Battery_Recharge):
        segment.state.conditions.energy.recharging  = True 
        segment.state.unknowns['recharge']          =  0* ones_row(1)  
        segment.state.residuals.network['recharge'] =  0* ones_row(1)
        segment.state.number_of_unknowns  += 1
        segment.state.number_of_residuals += 1    
    elif type(segment) == RCAIDE.Framework.Mission.Segments.Ground.Battery_Discharge:
        segment.state.conditions.energy.recharging   = False 
        segment.state.unknowns['discharge']          =  0* ones_row(1)  
        segment.state.residuals.network['discharge'] =  0* ones_row(1) 
        segment.state.number_of_unknowns  += 1
        segment.state.number_of_residuals += 1    
    else:
        segment.state.conditions.energy.recharging  = False  
        
    # This is the only one besides energy and discharge flag that should be moduleed into the segment top level
    if 'increment_battery_age_by_one_day' not in segment:
        segment.increment_battery_age_by_one_day   = False    
     
    return 
    
def append_battery_segment_conditions(battery, segment): 
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

    module_conditions = segment.state.conditions.energy.sources[battery.tag]    
    module_conditions.inputs.power.electrical[:,0]    = 0.0   
    module_conditions.outputs.power.electrical[:,0]   = 0.0  
    
    if segment.state.initials:   
        battery_initials                                        = segment.state.initials.conditions.energy.sources[battery.tag]   
        if type(segment) ==  RCAIDE.Framework.Mission.Segments.Ground.Battery_Recharge:             
            module_conditions.battery_discharge_flag      = False 
        else:                      
            module_conditions.battery_discharge_flag      = True      
            
        module_conditions.energy[:,0]                     = battery_initials.energy[-1,0]
        module_conditions.temperature[:,0]                = battery_initials.temperature[-1,0]
        
    if 'battery_cell_temperature' in segment:       
        module_conditions.temperature[:,0]                = segment.battery_cell_temperature  

    return    