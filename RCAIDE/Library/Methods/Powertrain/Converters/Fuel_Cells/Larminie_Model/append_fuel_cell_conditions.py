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
from RCAIDE.Library.Methods.Powertrain.Converters.Common.append_converter_power_conditions import append_converter_power_conditions

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

    fuel_cell_conditions                                           = append_converter_power_conditions(fuel_cell_stack, segment)
    fuel_cell_conditions.power                                     = 0 * ones_row(1)
    fuel_cell_conditions.voltage_under_load                        = 0 * ones_row(1)
    fuel_cell_conditions.current                                   = 0 * ones_row(1)
    fuel_cell_conditions.voltage_open_circuit                      = 0 * ones_row(1)
    fuel_cell_conditions.H2_mass_flow_rate                         = 0 * ones_row(1)
    
    # Conditions for recharging fuel_cell
    if isinstance(segment,RCAIDE.Framework.Mission.Segments.Ground.Battery_Recharge):
        segment.state.conditions.energy.recharging  = True
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
    fuel_cell_conditions = segment.state.conditions.energy.converters[fuel_cell_stack.tag]
    fuel_cell_conditions.inputs.power.electrical[:,0]  = 0.0
    fuel_cell_conditions.inputs.power.chemical[:,0]    = 0.0
    fuel_cell_conditions.outputs.power.electrical[:,0] = 0.0
    fuel_cell_conditions.outputs.power.chemical[:,0]   = 0.0
    return
  
def reuse_stored_fuel_cell_data(fuel_cell_stack,state,network,stored_converter_tag):
    '''Reuses results from one converter for identical fuel cells'''
    stored_conditions    = state.conditions.energy.converters[stored_converter_tag]
    fuel_cell_conditions = state.conditions.energy.converters[fuel_cell_stack.tag]
    fuel_cell_conditions.update(deepcopy(stored_conditions))

    return fuel_cell_conditions.inputs, fuel_cell_conditions.outputs