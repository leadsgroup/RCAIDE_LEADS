# RCAIDE/Methods/Powertrain/Sources/Batteries/Lithium_Ion_NMC/compute_nmc_cell_performance.py
# 
# 
# Created:  Feb 2024, M. Clarke
# Modified: Sep 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units 
import numpy as np
from copy import deepcopy
 
# ----------------------------------------------------------------------------------------------------------------------
# compute_nmc_cell_performance
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_nmc_cell_performance(battery_module,battery,state,network):
    """
    Computes the performance of a lithium-nickel-manganese-cobalt-oxide (NMC) battery cell.

    Parameters
    ----------
    battery_module : RCAIDE.Library.Components.Sources.Batteries.Modules.Lithium_Ion_NMC
        Battery module component with the following attributes:
            - tag : str
                Identifier for the battery module
            - cell : Data
                Cell properties
                    - electrode_area : float
                        Area of the electrode [m²]
                    - surface_area : float
                        Surface area of the cell [m²]
                    - mass : float
                        Mass of a single cell [kg]
                    - specific_heat_capacity : float
                        Specific heat capacity of the cell [J/(kg·K)]
                    - discharge_performance_map : function
                        Function that maps state of charge, temperature, and current to voltage
            - maximum_energy : float
                Maximum energy capacity of the module [J]
            - electrical_configuration : Data
                Electrical configuration
                    - series : int
                        Number of cells in series
                    - parallel : int
                        Number of cells in parallel
    state : RCAIDE.Framework.Mission.Common.State
        State object containing:
            - conditions : Data
                Flight conditions
                    - energy : dict
                        Energy conditions indexed by component tag
                            - [bus.tag] : Data
                                Bus-specific conditions
                                    - energy : numpy.ndarray
                                        Energy stored in the bus [J] 
                                        Current draw on the bus [A]
                                    - battery_modules : dict
                                        Battery module conditions indexed by tag
                                            - [battery_module.tag] : Data
                                                Battery module conditions
                                                    - energy : numpy.ndarray
                                                        Energy stored in the module [J]
                                                    - voltage_open_circuit : numpy.ndarray
                                                        Open-circuit voltage [V]
                                                    - power : numpy.ndarray
                                                        Power output [W]
                                                    - internal_resistance : numpy.ndarray
                                                        Internal resistance [Ω]
                                                    - heat_energy_generated : numpy.ndarray
                                                        Heat energy generated [W]
                                                    - voltage_under_load : numpy.ndarray
                                                        Voltage under load [V]
                                                    - current : numpy.ndarray
                                                        Current [A]
                                                    - temperature : numpy.ndarray
                                                        Temperature [K]
                                                    - state_of_charge : numpy.ndarray
                                                        State of charge [0-1]
                                                    - cell : Data
                                                        Cell-specific conditions with same properties as module
            - numerics : Data
                Numerical properties
                    - number_of_control_points : int
                        Number of control points in the mission
    bus : RCAIDE.Library.Components.Systems.Electrical_Bus
        Electrical bus component with the following attributes:
            - tag : str
                Identifier for the electrical bus
            - battery_module_electric_configuration : str
                Configuration of battery modules ("Series" or "Parallel")
            - battery_modules : list
                List of battery modules connected to the bus
    coolant_lines : list
        List of coolant lines for thermal management
    t_idx : int
        Current time index in the simulation
    delta_t : numpy.ndarray
        Time step size [s]

    Returns
    -------
    stored_results_flag : bool
        Flag indicating if results were stored
    stored_battery_module_tag : str
        Tag of the battery module for which results were stored

    Notes
    -----
    This function models the electrical and thermal behavior of an NMC battery cell
    based on experimental data. It updates various battery conditions in the `state` object,
    including: current energy, temperature, heat energy generated, load power, current,
    open-circuit voltage, charge throughput, internal resistance, state of charge,
    depth of discharge, and voltage under load.

    The model includes:
        - Internal resistance calculation
        - Thermal modeling (heat generation and temperature change)
        - Electrical performance (voltage and current calculations)
        - State of charge and depth of discharge updates

    **Major Assumptions**
        * All battery modules exhibit the same thermal behavior
        * The cell temperature is assumed to be the temperature of the entire module
        * Battery performance follows empirical models based on experimental data

    **Theory**
    The internal resistance is modeled as a function of state of charge:
    
    .. math::
        R_0 = 0.01483 \\cdot SOC^2 - 0.02518 \\cdot SOC + 0.1036
    
    Heat generation includes both Joule heating and entropy effects:
    
    .. math::
        \\dot{q}_{entropy} = -T \\cdot \\Delta S \\cdot i / (nF)
        
        \\dot{q}_{joule} = i^2 / \\sigma
        
        Q_{heat} = (\\dot{q}_{joule} + \\dot{q}_{entropy}) \\cdot A_s

    References
    ----------
    [1] Zou, Y., Hu, X., Ma, H., and Li, S. E., "Combined State of Charge and State of Health estimation over lithium-ion battery cell cycle lifespan for electric vehicles," Journal of Power Sources, Vol. 273, 2015, pp. 793-803. doi:10.1016/j.jpowsour.2014.09.146
    [2] Jeon, D. H., and Baek, S. M., "Thermal modeling of cylindrical lithium ion battery during discharge cycle," Energy Conversion and Management, Vol. 52, No. 8-9, 2011, pp. 2973-2981. doi:10.1016/j.enconman.2011.04.013

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Lithium_Ion_LFP
    """

    # ---------------------------------------------------------------------------------     
    # Time discretization
    # --------------------------------------------------------------------------------- 
    D = state.numerics.time.differentiate
    I = state.numerics.time.integrate
    
    # ---------------------------------------------------------------------------------    
    # battery cell properties
    # --------------------------------------------------------------------------------- 
    electrode_area            = battery_module.cell.electrode_area 
    As_cell                   = battery_module.cell.surface_area
    cell_mass                 = battery_module.cell.mass    
    Cp                        = battery_module.cell.specific_heat_capacity       
    battery_module_data       = battery_module.cell.discharge_performance_map
    
    # ---------------------------------------------------------------------------------
    # Compute Bus electrical properties 
    # ---------------------------------------------------------------------------------    
    #bus_conditions              = state.conditions.energy.sources[battery]
    #bus_config                  = battery.battery_module_electric_configuration
    #psi                         = state.conditions.energy.battery_fuel_cell_power_split_ratio
    # ---------------------------------------------------------------------------------
    # Compute battery_module Conditions
    # ---------------------------------------------------------------------------------
    battery_module_conditions = state.conditions.energy.sources[battery.tag][battery_module.tag]  
    P_module                    = battery_module_conditions.power_draw 
    I_module                    = battery_module_conditions.current_draw 
     
   
    E_module_max       = battery_module.maximum_energy * battery_module_conditions.cell.capacity_fade_factor 
    V_oc_module        = battery_module_conditions.voltage_open_circuit
    V_oc_cell          = battery_module_conditions.cell.voltage_open_circuit    
    #P_module           = battery_module_conditions.power
    P_cell             = battery_module_conditions.cell.power 
    R_0_module         = battery_module_conditions.internal_resistance
    R_0_cell           = battery_module_conditions.cell.internal_resistance 
    Q_heat_module      = battery_module_conditions.heat_energy_generated
    Q_heat_cell        = battery_module_conditions.cell.heat_energy_generated 
    V_ul_cell          = battery_module_conditions.cell.voltage_under_load 
    #I_module           = battery_module_conditions.current 
    I_cell             = battery_module_conditions.cell.current

    # ---------------------------------------------------------------------------------                   
    # set unknowns 
    # ---------------------------------------------------------------------------------
    T_cell_unkn   = state.unknowns.network[battery.tag + '_' + battery_module.tag  + '_cell_temperature']
    SOC_cell_unkn = state.unknowns.network[battery.tag + '_' + battery_module.tag  + '_cell_state_of_charge']
    
    # ---------------------------------------------------------------------------------
    # Electrical configuration
    # ---------------------------------------------------------------------------------
    n_series   = battery_module.electrical_configuration.series
    n_parallel = battery_module.electrical_configuration.parallel 
    n_total    = n_series*n_parallel 
    #no_modules = len(bus.battery_modules)
    
    # Scaling factors for numerical conditioning
    T_scale = 310.0
    E_scale = E_module_max
    
    # Scaled and bounded unknowns
    T_cell_scaled = T_cell_unkn / T_scale
    SOC_bounded = np.clip(SOC_cell_unkn, 1e-4, 1.0)
    
    
    # ---------------------------------------------------------------------------------
    # Compute Bus electrical properties 
    # --------------------------------------------------------------------------------- 
    coolant_line = None  
    if battery_module.assigned_distributors != None: 
        for distributor_tag in battery_module.assigned_distributors[0]: 
            distributor   = network.distributors(distributor_tag) 
            if type(distributor) == RCAIDE.Library.Components.Powertrain.Distributors.Coolant_Line:
                coolant_line = distributor 
    # ---------------------------------------------------------------------------------
    # Current calculations
    # ---------------------------------------------------------------------------------
    #if bus_config == 'Series':
        #I_module = I_bus
    #elif bus_config == 'Parallel':
        #I_module = I_bus / len(bus.battery_modules)
    I_cell = I_module / n_parallel
    
    # ---------------------------------------------------------------------------------
    # Resistance with logarithmic transformation for stability
    # ---------------------------------------------------------------------------------~
    log_R0_base = np.log(0.1036)
    R0_factor   = 0.01483/0.1036 * (SOC_bounded**2) - 0.02518/0.1036 * SOC_bounded
    R_0_cell    = np.exp(log_R0_base + R0_factor) * battery_module_conditions.cell.resistance_growth_factor
    R_0_cell    = np.maximum(R_0_cell, 1e-6)
    
    delta_S = -496.66*(SOC_bounded)**6 +  1729.4*(SOC_bounded)**5 + -2278 *(SOC_bounded)**4 +  \
                1382.2 *(SOC_bounded)**3 -380.47*(SOC_bounded)**2 +  46.508*(SOC_bounded)  + -10.692  
    
    # Heat generation
    sigma = 139
    n = 1
    F = 96485
    i_cell = I_cell / electrode_area
    
    q_dot_entropy = -(T_cell_unkn) * delta_S * i_cell / (n * F)
    q_dot_joule   = (i_cell**2) * battery_module_conditions.cell.resistance_growth_factor / sigma
    Q_heat_cell   = (q_dot_joule + q_dot_entropy) * As_cell
    Q_heat_module = Q_heat_cell * n_total
    
    # Voltage calculations with bounded inputs
    T_cell_bounded = np.clip(T_cell_unkn, 272.65, 322.65)
    V_ul_cell      = compute_nmc_cell_state(battery_module_data, SOC_bounded, T_cell_bounded, abs(I_cell))
    V_oc_cell      = V_ul_cell + (abs(I_cell) * R_0_cell)
    
    # Power calculations
    #P_module = P_bus / no_modules
    P_cell   = P_module / n_total 
    
    # Store electrical variables
    V_oc_module = V_oc_cell * n_series
    R_0_module  = (R_0_cell / n_parallel) * n_series

    if coolant_line != None and battery_module.heat_acquisition_system != None:
        HAS    = battery_module.heat_acquisition_system
        dT_dt  = HAS.battery_module.compute_thermal_performance(coolant_line,Q_heat_cell,T_cell_bounded,state)
    else:
        # Temperature residual with scaling
        dT_dt  = Q_heat_cell / (cell_mass * Cp)
    dT_dt_scaled     = dT_dt /  T_scale
    R_temp           = np.dot(D, T_cell_scaled)[:, 0] - dT_dt_scaled[:, 0]
    R_temp[0]        = T_cell_scaled[0,0] - battery_module_conditions.cell.temperature[0, 0] / T_scale
    state.residuals.network[battery_module.tag+ '_cell_temperature'] = R_temp
        
    # SOC residual with better conditioning
    dE_dt    = -P_module
    R_soc    = np.dot(D, SOC_cell_unkn * E_scale)[:, 0] - dE_dt[:, 0]
    R_soc[0] = SOC_cell_unkn[0,0] - battery_module_conditions.cell.state_of_charge[0, 0]
    state.residuals.network[battery_module.tag+ '_cell_state_of_charge'] = R_soc
    
    # Update states
    battery_module_conditions.voltage_under_load            = V_ul_cell * n_series
    battery_module_conditions.cell.voltage_under_load       = V_ul_cell

    battery_module_conditions.voltage_open_circuit          = V_oc_module
    battery_module_conditions.cell.voltage_open_circuit     = V_oc_cell

    battery_module_conditions.internal_resistance           = R_0_module
    battery_module_conditions.cell.internal_resistance      = R_0_cell
    
    battery_module_conditions.cell.power                    = P_cell
    battery_module_conditions.cell.current                  = I_cell
    battery_module_conditions.current                       = I_module


    battery_module_conditions.heat_energy_generated         = Q_heat_module
    battery_module_conditions.cell.heat_energy_generated    = Q_heat_cell

    battery_module_conditions.cell.state_of_charge[1:,0]    = SOC_cell_unkn[1:,0]
    battery_module_conditions.state_of_charge[1:,0]         = SOC_cell_unkn[1:,0]

    battery_module_conditions.cell.temperature[1:,0]        = T_cell_unkn[1:,0]
    battery_module_conditions.temperature[1:,0]             = T_cell_unkn[1:,0]

    battery_module_conditions.cell.depth_of_discharge[1:,0] = 1. - SOC_cell_unkn[1:,0]
    battery_module_conditions.cell.energy[1:,0]             = SOC_cell_unkn[1:,0] * E_module_max / n_total
    battery_module_conditions.energy[1:,0]                  = SOC_cell_unkn[1:,0] * E_module_max
    
    # Charge throughput
    Q_prior = battery_module_conditions.cell.charge_throughput[0]
    dt      = np.diff(state.numerics.time.control_points[:,0])
    avg_I   = (I_cell[:-1, 0] + I_cell[1:, 0]) / 2
    Q_Ah    = np.atleast_2d(np.concatenate(([0.0], np.cumsum(dt*avg_I)))).T / Units.hr
    battery_module_conditions.cell.charge_throughput = Q_prior + Q_Ah
    
    stored_results_flag = True
    stored_battery_module_tag = battery_module.tag
    
    return battery_module_conditions.inputs, battery_module_conditions.outputs, stored_results_flag, stored_battery_module_tag 

def reuse_stored_nmc_cell_data(battery_module,state,stored_battery_tag,stored_battery_module_tag):
    '''Reuses results from one propulsor for identical batteries
    
    Assumptions: 
    N/A

    Source:
    N/A

    Inputs:  
    

    Outputs:  
    
    Properties Used: 
    N.A.        
    '''
    
    state.conditions.energy.sources[battery_module.tag][stored_battery_tag] = deepcopy(state.conditions.energy.sources[stored_battery_module_tag])
 
        
    return  
 
def compute_nmc_cell_state(battery_module_data, SOC, T, I):
    """
    Computes the electrical state variables of a lithium-nickel-manganese-cobalt-oxide (NMC) battery cell using look-up tables.
    
    Parameters
    ----------
    battery_module_data : function
        Look-up function that maps state of charge, temperature, and current to voltage
    SOC : numpy.ndarray
        State of charge of the cell [unitless, 0-1]
    T : numpy.ndarray
        Battery cell temperature [K]
    I : numpy.ndarray
        Battery cell current [A]
    
    Returns
    -------
    V_ul : numpy.ndarray
        Under-load voltage [V]
    
    Notes
    -----
    This function computes the voltage of an NMC battery cell under load conditions
    by using a look-up table approach. It converts the state of charge to depth of discharge,
    and then uses this value along with temperature and current to determine the cell voltage.
    
    The function applies limits to ensure the inputs are within the valid range of the
    look-up data:
        - SOC is limited to [0, 1]
        - Temperature is limited to [272.65K, 322.65K] (approximately 0°C to 50°C)
        - Current is limited to [0A, 8A]
    
    The input to the look-up table is a concatenated array of [current, temperature, depth_of_discharge].
    
    **Major Assumptions**
        * The battery performance can be accurately represented by a look-up table
        * The model is valid only within the specified temperature and current ranges
    
    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Batteries.Modules.Lithium_Ion_NMC
    """

    # Make sure things do not break by limiting current, temperature and current 
    SOC[SOC < 0.]   = 0.  
    SOC[SOC > 1.]   = 1.    
    DOD             = 1 - SOC 
    
    T[np.isnan(T)] = 302.65
    T[T<272.65]    = 272.65 # model does not fit for below 0  degrees
    T[T>322.65]    = 322.65 # model does not fit for above 50 degrees
     
    I[I<0.0]       = 0.0
    I[I>8.0]       = 8.0   
     
    pts            = np.hstack((np.hstack((I, T)),DOD  )) # amps, temp, SOC   
    V_ul           = np.atleast_2d(battery_module_data.Voltage(pts)[:,1]).T  
    
    return V_ul