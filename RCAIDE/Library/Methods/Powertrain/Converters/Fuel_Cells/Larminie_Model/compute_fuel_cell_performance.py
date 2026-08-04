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
def compute_fuel_cell_performance(fuel_cell_stack, state, network):
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

    The power this stack must supply is its own electrical distributor's
    total demand, split between battery/fuel-cell sources by that
    distributor's psi and between multiple fuel-cell stacks on the same bus
    by ``power_split_ratio``. The distributor is looked up from
    ``fuel_cell_stack.assigned_distributors`` rather than assumed, since a
    fuel cell stack may share the vehicle with other electrically-isolated
    buses resolved to a different psi.

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
    n_total           = n_series*n_parallel

    # ---------------------------------------------------------------------------------
    # Compute fuel cell stack conditions
    # ---------------------------------------------------------------------------------
    fuel_cell_stack_conditions  = state.conditions.energy.converters[fuel_cell_stack.tag]

    electrical_distributor_tag = None
    for d_tag in fuel_cell_stack.assigned_distributors[0]:
        if network.distributors[d_tag].domain == 'electrical':
            electrical_distributor_tag = d_tag
    psi = state.conditions.energy.battery_fuel_cell_power_split_ratio[electrical_distributor_tag]
    total_electrical_demand = state.conditions.energy.distributors[electrical_distributor_tag].outputs.power.electrical
    P_stack                     = total_electrical_demand * fuel_cell_stack.power_split_ratio * (1. - psi)
    n_ctrl_pts                  = state.numerics.number_of_control_points

    for t_idx in range(n_ctrl_pts):
        P_cell = P_stack[t_idx, 0] / n_total

        # ---------------------------------------------------------------------------------
        # Compute fuel cell performance
        # ---------------------------------------------------------------------------------
        lb                          = 0.0001/(Units.cm**2.)
        ub                          = 1.2/(Units.cm**2.)
        current_density             = sp.optimize.fminbound(compute_power_difference, lb, ub, args=(fuel_cell,P_cell))
        V_fuel_cell                 = compute_voltage(fuel_cell,current_density)
        efficiency                  = np.divide(V_fuel_cell, fuel_cell.ideal_voltage)
        mdot_cell                   = np.divide(P_cell,np.multiply(fuel_cell.propellant.specific_energy,efficiency))

        I_cell  = P_cell / V_fuel_cell
        I_stack = I_cell * n_parallel

        fuel_cell_stack_conditions.power[t_idx]                                = P_stack[t_idx, 0]
        fuel_cell_stack_conditions.current[t_idx]                              = I_stack
        fuel_cell_stack_conditions.voltage_open_circuit[t_idx]                 = V_fuel_cell *  n_series
        fuel_cell_stack_conditions.voltage_under_load[t_idx]                   = V_fuel_cell *  n_series
        fuel_cell_stack_conditions.H2_mass_flow_rate[t_idx]                    = mdot_cell * n_total

    stored_results_flag                                               = True
    stored_converter_tag                                              = fuel_cell_stack.tag
    fuel_cell_stack_conditions.outputs.power.electrical               = fuel_cell_stack_conditions.power * fuel_cell_stack.electrical_efficiency

    # Chemical (hydrogen) power draw, fed to the fuel line so assigned fuel tanks
    # can compute their own mass depletion.
    fuel_cell_stack_conditions.inputs.power.chemical = fuel_cell_stack_conditions.H2_mass_flow_rate * fuel_cell.propellant.specific_energy
    fuel_cell_stack_conditions.fuel_mass_flow_rate    = fuel_cell_stack_conditions.H2_mass_flow_rate

    return  fuel_cell_stack_conditions.inputs, fuel_cell_stack_conditions.outputs, stored_results_flag, stored_converter_tag


