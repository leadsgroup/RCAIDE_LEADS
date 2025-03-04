# RCAIDE/Library/Methods/Powertrain/Converters/Fuel_Cells/Common/compute_fuel_cell_performance.py 
# 
# Created: Jan 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Larminie_Model   import compute_voltage, compute_power_difference

import numpy as np
import scipy as sp

# ----------------------------------------------------------------------
#  Larminie Model to Compute Fuel Cell Performance
# ---------------------------------------------------------------------- 
def compute_fuel_cell_performance(fuel_cell_stack,state,bus,coolant_lines,t_idx,delta_t): 
    """
    Computes the performance of a fuel cell using Larminie model
    
     Parameters
    ----------
    fuel_cell_stack : fuel_cell_stack
        The fuel_cell_stack object containing cell properties and configuration.
    state : MissionState
        The current state of the mission.
    bus : ElectricBus
        The electric bus to which the battery_module is connected.
    coolant_lines : list
        List of coolant lines for thermal management.
    t_idx : int
        Current time index in the simulation.
    delta_t : float
        Time step size.
         
    Returns: 
    ----------
    stored_results_flag: bool
        Stored results boolean
    stored_fuel_cell_stack_tag: string 
        Tag of fuel cell
    """
    # ---------------------------------------------------------------------------------    
    # fuel cell stack properties 
    # ---------------------------------------------------------------------------------
    fuel_cell         = fuel_cell_stack.fuel_cell 
    n_series          = fuel_cell_stack.electrical_configuration.series
    n_parallel        = fuel_cell_stack.electrical_configuration.parallel 
    bus_config        = bus.fuel_cell_stack_electric_configuration
    n_total           = n_series*n_parallel  
        
    # ---------------------------------------------------------------------------------
    # Compute Bus electrical properties   
    # ---------------------------------------------------------------------------------
    bus_conditions              = state.conditions.energy[bus.tag]
    fuel_cell_stack_conditions  = bus_conditions.fuel_cell_stacks[fuel_cell_stack.tag]
    P_bus                       = bus_conditions.power_draw[t_idx] 
     
    
    P_stack  = P_bus /len(bus.fuel_cell_stacks) 
    P_cell   = P_stack / n_total  

    # ---------------------------------------------------------------------------------
    # Compute fuel cell performance  
    # ---------------------------------------------------------------------------------
    lb                          = 0.0001/(Units.cm**2.)    # lower bound on fuel cell current density 
    ub                          = 1.2/(Units.cm**2.)       # upper bound on fuel cell current density
    current_density             = sp.optimize.fminbound(compute_power_difference, lb, ub, args=(fuel_cell,P_cell)) 
    V_fuel_cell                 = compute_voltage(fuel_cell,current_density)    
    efficiency                  = np.divide(V_fuel_cell, fuel_cell.ideal_voltage)
    mdot_cell                   = np.divide(P_cell,np.multiply(fuel_cell.propellant.specific_energy,efficiency)) 
    
    I_cell = P_cell / V_fuel_cell
    I_stack = I_cell * n_parallel
    if bus_config == 'Series':
        bus_conditions.current_draw[t_idx] = I_stack  
    elif bus_config  == 'Parallel': 
        bus_conditions.current_draw[t_idx] = I_stack * len(bus.fuel_cell_stacks)  
    
    fuel_cell_stack_conditions.power[t_idx]                                = P_stack
    fuel_cell_stack_conditions.current[t_idx]                              = I_stack
    fuel_cell_stack_conditions.voltage_open_circuit[t_idx]                 = V_fuel_cell *  n_series # assumes no losses
    fuel_cell_stack_conditions.voltage_under_load[t_idx]                   = V_fuel_cell *  n_series
    fuel_cell_stack_conditions.fuel_cell.voltage_open_circuit[t_idx]       = V_fuel_cell   # assumes no losses
    fuel_cell_stack_conditions.fuel_cell.voltage_under_load[t_idx]         = V_fuel_cell
    fuel_cell_stack_conditions.fuel_cell.power[t_idx]                      = P_cell
    fuel_cell_stack_conditions.fuel_cell.current[t_idx]                    = P_cell / V_fuel_cell 
    fuel_cell_stack_conditions.fuel_cell.inlet_H2_mass_flow_rate[t_idx]    = mdot_cell  
    fuel_cell_stack_conditions.H2_mass_flow_rate[t_idx]                    = mdot_cell * n_total
    
    stored_results_flag            = True
    stored_fuel_cell_stack_tag     = fuel_cell_stack.tag  

    return  stored_results_flag, stored_fuel_cell_stack_tag


