# RCAIDE/Methods/Powertrain/Sources/Fuel_Cells/Larminie_Nodel/append_fuel_cell_conditions.py
# 
# 
# Created: Nov 2024, M. Clarke
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE 
from RCAIDE.Framework.Mission.Common     import   Conditions

# pack imports 
from copy import  deepcopy

# ----------------------------------------------------------------------------------------------------------------------
# append_fuel_cell_conditions
# ----------------------------------------------------------------------------------------------------------------------  
def append_fuel_cell_conditions(fuel_cell_stack,segment):
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
    """      
    
    ones_row = segment.state.ones_row  
                                             
    fuel_cell_conditions                                           = segment.state.conditions.energy.converters[fuel_cell_stack.tag]
    fuel_cell_conditions                                           = Conditions()
    fuel_cell_conditions                                           = Conditions()
    fuel_cell_conditions.inputs                                    = Conditions()
    fuel_cell_conditions.outputs                                   = Conditions()
    fuel_cell_conditions.cell                                      = Conditions()
    fuel_cell_conditions.inputs.power                              = Conditions()
    fuel_cell_conditions.outputs.power                             = Conditions()
    fuel_cell_conditions.inputs.power.propulsive                   = 0 * ones_row(1)
    fuel_cell_conditions.inputs.power.mechanical                   = 0 * ones_row(1)
    fuel_cell_conditions.inputs.power.electrical                   = 0 * ones_row(1)
    fuel_cell_conditions.inputs.power.chemical                     = 0 * ones_row(1)
    fuel_cell_conditions.inputs.power.pneumatic                    = 0 * ones_row(1)
    fuel_cell_conditions.inputs.power.hydraulic                    = 0 * ones_row(1)
    fuel_cell_conditions.inputs.power.thermal                      = 0 * ones_row(1)
    fuel_cell_conditions.outputs.power.propulsive                  = 0 * ones_row(1)
    fuel_cell_conditions.outputs.power.mechanical                  = 0 * ones_row(1)
    fuel_cell_conditions.outputs.power.electrical                  = 0 * ones_row(1)
    fuel_cell_conditions.outputs.power.chemical                    = 0 * ones_row(1)
    fuel_cell_conditions.outputs.power.pneumatic                   = 0 * ones_row(1)
    fuel_cell_conditions.outputs.power.hydraulic                   = 0 * ones_row(1)
    fuel_cell_conditions.outputs.power.thermal                     = 0 * ones_row(1)
    fuel_cell_conditions.voltage_under_load                        = 0 * ones_row(1)
    fuel_cell_conditions.current                                   = 0 * ones_row(1)  
    fuel_cell_conditions.voltage_open_circuit                      = 0 * ones_row(1) 
    fuel_cell_conditions.fuel_cell.voltage_open_circuit            = 0 * ones_row(1)  
    fuel_cell_conditions.fuel_cell.voltage_under_load              = 0 * ones_row(1)  
    fuel_cell_conditions.fuel_cell.current                         = 0 * ones_row(1)  
    fuel_cell_conditions.fuel_cell.inlet_H2_mass_flow_rate         = 0 * ones_row(1)
    fuel_cell_conditions.fuel_cell.inlet_air_mass_flow_rate        = 0 * ones_row(1) 
    fuel_cell_conditions.H2_mass_flow_rate                         = 0 * ones_row(1) 
    
    # Conditions for recharging fuel_cell 
    if isinstance(segment,RCAIDE.Framework.Mission.Segments.Ground.Battery_Recharge):
        segment.state.conditions.energy.recharging  = True 
        segment.state.unknowns.mission['recharge']          =  0* ones_row(1)  
        segment.state.residuals.mission.network['recharge'] =  0* ones_row(1)
        segment.state.number_of_mission_unknowns  += 1
        segment.state.number_of_mission_residuals += 1    
    elif type(segment) == RCAIDE.Framework.Mission.Segments.Ground.Battery_Discharge:
        segment.state.conditions.energy.recharging   = False 
        segment.state.unknowns.mission['discharge']          =  0* ones_row(1)  
        segment.state.residuals.mission.network['discharge'] =  0* ones_row(1) 
        segment.state.number_of_mission_unknowns  += 1
        segment.state.number_of_mission_residuals += 1        
    else:
        segment.state.conditions.energy.recharging  = False             
    return
 
def append_fuel_cell_segment_conditions(fuel_cell_stack, segment): 
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
    fuel_cell_conditions = segment.state.conditions[bus.tag].fuel_cell_stacks[fuel_cell_stack.tag]
    if segment.state.initials:  
        fuel_cell_initials                                   = segment.state.initials.conditions.energy.busses[bus.tag].fuel_cell_stacks[fuel_cell_stack.tag]
        fuel_cell_conditions.temperature[:,0]                = fuel_cell_initials.temperature[-1,0]
        fuel_cell_conditions.cell.temperature[:,0]           = fuel_cell_initials.cell.temperature[-1,0]     
    return
  
def reuse_stored_fuel_cell_data(fuel_cell_stack,state,bus,stored_results_flag, stored_fuel_cell_stack_tag):
    '''Reuses results from one propulsor for identical fuel cells     
    ''' 
    state.conditions.energy.busses[bus.tag].fuel_cell_stacks[fuel_cell_stack.tag] = deepcopy(state.conditions.energy.busses[bus.tag].fuel_cell_stacks[stored_fuel_cell_stack_tag])
     
    return