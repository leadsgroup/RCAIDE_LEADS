# RCAIDE/Methods/Powertrain/Sources/Batteries/Lithium_Ion_NMC/compute_nmc_cell_performance.py
# 
# 
# Created:  Feb 2024, M. Clarke
# Modified: Sep 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core                       import Units 
import numpy as np
from copy import  deepcopy
 
# ----------------------------------------------------------------------------------------------------------------------
# compute_nmc_cell_performance
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_nmc_cell_performance(battery_module,state,bus,coolant_lines,t_idx, delta_t): 
    """
    Compute the performance of a lithium-nickel-manganese-cobalt-oxide (NMC) battery_module cell.

    This function models the electrical and thermal behavior of an 18650 NMC battery_module cell
    based on experimental data from the Automotive Industrial Systems Company of Panasonic Group.

    Parameters
    ----------
    battery_module : battery_module
        The battery_module object containing cell properties and configuration.
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

    Returns
    -------
    tuple
        A tuple containing:
        - stored_results_flag (bool): Flag indicating if results were stored.
        - stored_battery_module_tag (str): Tag of the battery_module for which results were stored.

    Notes
    -----
    The function updates various battery_module conditions in the `state` object, including:
    - Current energy
    - Temperature
    - Heat energy generated
    - Load power
    - Current
    - Open-circuit voltage
    - Charge throughput
    - Internal resistance
    - State of charge
    - Depth of discharge
    - Voltage under load

    The model includes:
    - Internal resistance calculation
    - Thermal modeling (heat generation and temperature change)
    - Electrical performance (voltage and current calculations)
    - State of charge and depth of discharge updates

    Arrays accessed from objects:
    - From bus_conditions:
        - power_draw
        - current_draw
    - From battery_module_conditions:
        - energy
        - voltage_open_circuit
        - cell.voltage_open_circuit
        - power
        - cell.power
        - internal_resistance
        - cell.internal_resistance
        - heat_energy_generated
        - cell.heat_energy_generated
        - voltage_under_load
        - cell.voltage_under_load
        - current
        - cell.current
        - temperature
        - cell.temperature
        - cell.state_of_charge
        - cell.energy
        - cell.charge_throughput
        - cell.depth_of_discharge

    References
    ----------
    .. [1] Zou, Y., Hu, X., Ma, H., and Li, S. E., "Combined State of Charge and State of
           Health estimation over lithium-ion battery_module cell cycle lifespan for electric 
           vehicles," Journal of Power Sources, Vol. 273, 2015, pp. 793-803.
           doi:10.1016/j.jpowsour.2014.09.146
    .. [2] Jeon, Dong Hyup, and Seung Man Baek. "Thermal modeling of cylindrical lithium ion 
           battery_module during discharge cycle." Energy Conversion and Management 52.8-9 (2011): 
           2973-2981.

    Assumptions
    -----------
    - All battery_module modules exhibit the same thermal behavior.
    - The cell temperature is assumed to be the temperature of the entire module.
    """

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
    bus_conditions              = state.conditions.energy[bus.tag]
    bus_config                  = bus.battery_module_electric_configuration
    E_bus                       = bus_conditions.energy
    P_bus                       = bus_conditions.power_draw
    I_bus                       = bus_conditions.current_draw
    
    # ---------------------------------------------------------------------------------
    # Compute battery_module Conditions
    # -------------------------------------------------------------------------    
    battery_module_conditions = state.conditions.energy[bus.tag].battery_modules[battery_module.tag]  
   
    E_module_max       = battery_module.maximum_energy * battery_module_conditions.cell.capacity_fade_factor
    
    V_oc_module        = battery_module_conditions.voltage_open_circuit
    V_oc_cell          = battery_module_conditions.cell.voltage_open_circuit   
  
    P_module           = battery_module_conditions.power
    P_cell             = battery_module_conditions.cell.power
    
    R_0_module         = battery_module_conditions.internal_resistance
    R_0_cell           = battery_module_conditions.cell.internal_resistance
    
    Q_heat_module      = battery_module_conditions.heat_energy_generated
    Q_heat_cell        = battery_module_conditions.cell.heat_energy_generated
    
    V_ul_module        = battery_module_conditions.voltage_under_load
    V_ul_cell          = battery_module_conditions.cell.voltage_under_load
    
    I_module           = battery_module_conditions.current 
    I_cell             = battery_module_conditions.cell.current
    
    T_module           = battery_module_conditions.temperature                 
    T_cell             = battery_module_conditions.cell.temperature
    
    SOC_cell           = battery_module_conditions.cell.state_of_charge  
    SOC_module         = battery_module_conditions.state_of_charge
    E_cell             = battery_module_conditions.cell.energy   
    E_module           = battery_module_conditions.energy
    Q_cell             = battery_module_conditions.cell.charge_throughput              
    DOD_cell           = battery_module_conditions.cell.depth_of_discharge
    
    # ---------------------------------------------------------------------------------
    # Compute battery_module electrical properties 
    # -------------------------------------------------------------------------    
    # Calculate the current going into one cell  
    n_series          = battery_module.electrical_configuration.series
    n_parallel        = battery_module.electrical_configuration.parallel 
    n_total           = n_series*n_parallel 
    no_modules        = len(bus.battery_modules)
    
    # ---------------------------------------------------------------------------------
    # Examine Thermal Management System
    # ---------------------------------------------------------------------------------
    HAS = None  
    for coolant_line in coolant_lines:
        for tag, item in  coolant_line.items():
            if tag == 'battery_modules':
                for sub_tag, sub_item in item.items():
                    if sub_tag == battery_module.tag:
                        for btms in  sub_item:
                            HAS = btms    


    # ---------------------------------------------------------------------------------------------------
    # Current State 
    # ---------------------------------------------------------------------------------------------------
    if bus_config == 'Series':
        I_module[t_idx]      = I_bus[t_idx]
    elif bus_config  == 'Parallel':
        I_module[t_idx]      = I_bus[t_idx] /len(bus.battery_modules)

    I_cell[t_idx] = I_module[t_idx] / n_parallel   
       
    # ---------------------------------------------------------------------------------
    # Compute battery_module cell temperature 
    # ---------------------------------------------------------------------------------
    R_0_cell[t_idx]                     =  (0.01483*(SOC_cell[t_idx]**2) - 0.02518*SOC_cell[t_idx] + 0.1036) *battery_module_conditions.cell.resistance_growth_factor  
    R_0_cell[t_idx][R_0_cell[t_idx]<0]  = 0. 

    # Determine temperature increase         
    sigma                 = 139 # Electrical conductivity
    n                     = 1
    F                     = 96485 # C/mol Faraday constant    
    delta_S               = -496.66*(SOC_cell[t_idx])**6 +  1729.4*(SOC_cell[t_idx])**5 + -2278 *(SOC_cell[t_idx])**4 +  1382.2 *(SOC_cell[t_idx])**3 + \
                            -380.47*(SOC_cell[t_idx])**2 +  46.508*(SOC_cell[t_idx])  + -10.692  

    i_cell                = I_cell[t_idx]/electrode_area # current intensity
    q_dot_entropy         = -(T_cell[t_idx])*delta_S*i_cell/(n*F)       
    q_dot_joule           = (i_cell**2)*(battery_module_conditions.cell.resistance_growth_factor)/(sigma)          
    Q_heat_cell[t_idx]    = (q_dot_joule + q_dot_entropy)*As_cell 
    Q_heat_module[t_idx]  = Q_heat_cell[t_idx]*n_total  

    V_ul_cell[t_idx]      = compute_nmc_cell_state(battery_module_data,SOC_cell[t_idx],T_cell[t_idx],abs(I_cell[t_idx])) 

    V_oc_cell[t_idx]      = V_ul_cell[t_idx] + (abs(I_cell[t_idx]) * R_0_cell[t_idx])              

    # Effective Power flowing through battery_module 
    P_module[t_idx]       = P_bus[t_idx] /no_modules  - np.abs(Q_heat_module[t_idx]) 

    # store remaining variables 
    V_oc_module[t_idx]     = V_oc_cell[t_idx]*n_series 
    V_ul_module[t_idx]     = V_ul_cell[t_idx]*n_series  
    T_module[t_idx]        = T_cell[t_idx]   # Assume the cell temperature is the temperature of the module
    P_cell[t_idx]          = P_module[t_idx]/n_total 
    E_module[t_idx]        = E_bus[t_idx]/no_modules 
    E_cell[t_idx]          = E_module[t_idx]/n_total  

    # ---------------------------------------------------------------------------------------------------     
    # Future State 
    # --------------------------------------------------------------------------------------------------- 
    if t_idx != state.numerics.number_of_control_points-1:  

        # Compute cell temperature
        if HAS is not None:
            T_cell[t_idx+1]  = HAS.compute_thermal_performance(battery_module,bus,coolant_line,Q_heat_cell[t_idx],T_cell[t_idx],state,delta_t[t_idx],t_idx)
        else:
            # Considers a thermally insulated system and the heat piles on in the system
            dT_dt              = Q_heat_cell[t_idx]/(cell_mass*Cp)
            T_cell[t_idx+1]    =  T_cell[t_idx] + dT_dt*delta_t[t_idx]
            
        # Compute state of charge and depth of discarge of the battery_module
        E_module[t_idx+1]                                     = (E_module[t_idx]) -P_module[t_idx]*delta_t[t_idx]
        E_module[t_idx+1][E_module[t_idx+1] > E_module_max]   = np.float32(E_module_max)
        SOC_cell[t_idx+1]                                     = E_module[t_idx+1]/E_module_max 
        SOC_cell[t_idx+1][SOC_cell[t_idx+1]>1]                = 1.
        SOC_cell[t_idx+1][SOC_cell[t_idx+1]<0]                = 0. 
        DOD_cell[t_idx+1]                                     = 1 - SOC_cell[t_idx+1]  
        SOC_module[t_idx+1]                                   = SOC_cell[t_idx+1]

    
        # Determine new charge throughput (the amount of charge gone through the battery_module)
        Q_cell[t_idx+1]    = Q_cell[t_idx] + abs(I_cell[t_idx])*delta_t[t_idx]/Units.hr
        
    stored_results_flag     = True
    stored_battery_module_tag     = battery_module.tag  
        
    return stored_results_flag, stored_battery_module_tag


def reuse_stored_nmc_cell_data(battery_module,state,bus,stored_results_flag, stored_battery_module_tag):
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
   
    state.conditions.energy[bus.tag].battery_modules[battery_module.tag] = deepcopy(state.conditions.energy[bus.tag].battery_modules[stored_battery_module_tag])
    
        
    return
 
def compute_nmc_cell_state(battery_module_data,SOC,T,I):
    """This computes the electrical state variables of a lithium ion 
    battery_module cell with a  lithium-nickel-cobalt-aluminum oxide cathode 
    chemistry from look-up tables 
     
    Assumtions: 
    N/A
    
    Source:  
    N/A 
     
    Inputs:
        SOC           - state of charge of cell     [unitless]
        battery_module_data  - look-up data structure      [unitless]
        T             - battery_module cell temperature    [Kelvin]
        I             - battery_module cell current        [Amperes]
    
    Outputs:  
        V_ul          - under-load voltage          [Volts] 
        
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