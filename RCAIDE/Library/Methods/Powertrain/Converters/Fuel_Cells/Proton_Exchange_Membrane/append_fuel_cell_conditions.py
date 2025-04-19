# RCAIDE/Methods/Powertrain/Sources/Fuel_Cells/Proton_Exchange_Membrane/append_fuel_cell_conditions.py
#  
# Created: Jan 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE 
from RCAIDE.Framework.Mission.Common     import   Conditions

# pack imports 
from copy import deepcopy

# ----------------------------------------------------------------------------------------------------------------------
# append_fuel_cell_conditions
# ----------------------------------------------------------------------------------------------------------------------  
def append_fuel_cell_conditions(fuel_cell_stack,segment,bus): 
    """
    Appends the initial fuel_cell conditions. 

    Parameters
    ----------
    fuel_cell : fuel_cell
        The fuel_cell object containing cell properties and configuration.
    segment : MissionSegment
        The current mission segment. 
    bus : bus
        The electrical bus object.

    Returns
    ------- 

    Notes
    -----
    The function appends various fuel cell conditions in the `state` object, including: 
    - power                                      
    - voltage_under_load                         
    - voltage_open_circuit             
    - current_density                  
    - current                          
    - H2_mass_flow_rate
 
    References
    ---------- 

    Assumptions
    ----------- 
    """      
    
    ones_row                                                                                       = segment.state.ones_row    
                                             
    bus_conditions                                                                                 = segment.state.conditions.energy[bus.tag]        
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag]                                           = Conditions()

    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].power                                     = 0 * ones_row(1)
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].current                                   = 0 * ones_row(1)      
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].voltage_open_circuit                      = 0 * ones_row(1) 
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].voltage_under_load                        = 0 * ones_row(1) 
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].H2_mass_flow_rate                         = 0 * ones_row(1)
    
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell                                 = Conditions() 
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.voltage_open_circuit            = 0 * ones_row(1)
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.voltage_under_load              = 0 * ones_row(1)
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.power                           = 0 * ones_row(1)
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.current                         = 0 * ones_row(1)  
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.stagnation_temperature          = 0 * ones_row(1)
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.stagnation_pressure             = 0 * ones_row(1)
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.oxygen_relative_humidity        = 1 * ones_row(1)
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.degradation                     = 0 * ones_row(1)
    
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.pressure_drop                   = 0 * ones_row(1)  
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.air_excess_ratio                = 0 * ones_row(1) 
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.stack_temperature               = fuel_cell_stack.fuel_cell.stack_temperature * ones_row(1)  
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.inlet_H2_mass_flow_rate         = 0 * ones_row(1) 
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.inlet_air_mass_flow_rate        = 0 * ones_row(1)    
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.outlet_air_pressure             = 0 * ones_row(1) 
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.outlet_air_mass_flow_rate       = 0 * ones_row(1) 
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.current_density                 = 0 * ones_row(1) 

    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.humidifier                      = Conditions() 
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.humidifier.pressure_drop        = 0 * ones_row(1)  
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.compressor                      = Conditions()   
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.compressor_inlet_pressure       = 0 * ones_row(1)
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.compressor_pressure_ratio       = 0 * ones_row(1)
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.compressor_inlet_mass_flow_rate = 0 * ones_row(1)
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.compressor_power                = 0 * ones_row(1)
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.expander                        = Conditions()   
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.expander_outlet_mass_flow_rate  = 0 * ones_row(1)
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.expander_inlet_pressure         = 0 * ones_row(1)
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.expander_pressure_ratio         = 0 * ones_row(1)
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.expander_power                  = 0 * ones_row(1)
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.motor_compressor                = Conditions()   
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.motor_compressor_power          = 0 * ones_row(1)
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.expander_generator              = Conditions()   
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.expander_generator_power        = 0 * ones_row(1)
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.compressor_expander_module      = Conditions
    bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag].fuel_cell.compressor_expander_module_power= 0 * ones_row(1)

    # Conditions for recharging fuel_cell        
    if isinstance(segment,RCAIDE.Framework.Mission.Segments.Ground.Battery_Recharge):
        segment.state.conditions.energy.recharging  = True  
        segment.state.unknowns['recharge']          =  0* ones_row(1)  
        segment.state.residuals['recharge'] =  0* ones_row(1)
    elif type(segment) == RCAIDE.Framework.Mission.Segments.Ground.Battery_Discharge:
        segment.state.conditions.energy.recharging   = False     
        segment.state.unknowns['recharge']          =  0* ones_row(1)  
        segment.state.residuals['recharge'] =  0* ones_row(1)
    else:
        segment.state.conditions.energy.recharging  = False            
    
    return

def append_fuel_cell_segment_conditions(fuel_cell_stack, bus, conditions, segment):  
    """
    Sets the initial fuel cell energy at the start of each segment as the last point from the previous segment
    
    Parameters
    ----------
    fuel_cell_stack : fuel_cell_stack
        The fuel_cell_stack object containing cell properties and configuration.
    bus : bus
        The electrical bus object.
    conditions : MissionConditions
        The current conditions of the mission segment segment
    segment : MissionSegment
        The current mission segment. 

    Returns
    ------- 
        
    """
    fuel_cell_conditions = conditions[bus.tag].fuel_cell_stacks[fuel_cell_stack.tag]
    if segment.state.initials:  
        fuel_cell_initials                                   = segment.state.initials.conditions.energy[bus.tag].fuel_cell_stacks[fuel_cell_stack.tag]    
        fuel_cell_conditions.temperature[:,0]                = fuel_cell_initials.temperature[-1,0]
        fuel_cell_conditions.cell.temperature[:,0]           = fuel_cell_initials.cell.temperature[-1,0]     
    return
  
def reuse_stored_fuel_cell_data(fuel_cell_stack,state,bus,stored_results_flag, stored_fuel_cell_stack_tag):
    '''
    Reuses results from one propulsor for identical fuel cells 
    '''
   
    state.conditions.energy[bus.tag].fuel_cell_stacks[fuel_cell_stack.tag] = deepcopy(state.conditions.energy[bus.tag].fuel_cell_stacks[stored_fuel_cell_stack_tag])
     
    return