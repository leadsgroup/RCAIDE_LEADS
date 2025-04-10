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
def compute_fuel_cell_performance(fuel_cell_stack, state, bus, coolant_lines, t_idx, delta_t):
    """
    Computes the performance of a fuel cell stack using the Larminie-Dicks model.
    
    Parameters
    ----------
    fuel_cell_stack : RCAIDE.Components.Energy.Converters.Fuel_Cell_Stack
        The fuel cell stack component containing cell properties and electrical configuration
    state : RCAIDE.Framework.Mission.Common.State
        Container for mission segment conditions
    bus : RCAIDE.Components.Energy.Distribution.Electric_Bus
        The electric bus to which the fuel cell stack is connected
    coolant_lines : list
        List of coolant line components for thermal management
    t_idx : int
        Current time index in the simulation
    delta_t : float
        Time step size [s]
         
    Returns
    -------
    stored_results_flag : bool
        Flag indicating that results have been stored for potential reuse
    stored_fuel_cell_stack_tag : str
        Tag identifier of the fuel cell stack with stored results
    
    Notes
    -----
    This function implements the Larminie-Dicks model to calculate fuel cell performance
    based on current operating conditions. It determines the optimal current density
    that matches the required power output, then calculates voltage, efficiency,
    and fuel consumption.
    
    The function handles both series and parallel electrical configurations for
    connecting the fuel cell stack to the electric bus.
    
    **Major Assumptions**
        * Uniform temperature distribution across all cells
        * No transient effects (steady-state operation at each time step)
        * Hydrogen is the only fuel considered
        * Ideal gas behavior
    
    **Theory**
    
    The Larminie-Dicks model calculates cell voltage as:
    
    .. math::
        V = E_0 - A\\ln(j) - Rj - m\\exp(nj)
    
    where:
        - E₀ is the open circuit voltage
        - A is the activation loss coefficient
        - R is the ohmic resistance
        - m and n are mass transport loss coefficients
        - j is the current density
    
    The efficiency is calculated as:
    
    .. math::
        \\eta = \\frac{V}{E_{ideal}}
    
    References
    ----------
    [1] Larminie, J., & Dicks, A. (2003). Fuel Cell Systems Explained (2nd ed.). John Wiley & Sons Ltd.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Larminie_Model.compute_voltage
    RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Larminie_Model.compute_power_difference
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
    phi                         = state.conditions.energy.hybrid_power_split_ratio 
    P_bus                       = bus_conditions.power_draw*phi     
    P_stack                     = P_bus[t_idx] /len(bus.fuel_cell_stacks) 
    P_cell                      = P_stack/ n_total  

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


