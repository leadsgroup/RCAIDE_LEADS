# RCAIDE/Methods/Powertrain/Sources/Batteries/Lithium_Ion_P30b/compute_p30b_cell_performance.py
# need to apply current electrical demand and store impact from previous time steps
# pybamm does cell level calculations. could I treat each module as a cell?
# Created:  Mar 2025, C. Boggan
#
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core                       import Units 
import numpy as np
from copy import  deepcopy
import pybamm

# ----------------------------------------------------------------------------------------------------------------------
# Create Base P30b battery model
# ---------------------------------------------------------------------------------------------------------------------- 




# ----------------------------------------------------------------------------------------------------------------------
# compute_p30b_cell_performance
# ---------------------------------------------------------------------------------------------------------------------- 

def compute_p30b_cell_performance(battery_module,state,bus,coolant_lines,t_idx, delta_t): 
    """
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

   
    """
    #these will be immutable. even parameter values do not change over the course of the experiment
   

    # ---------------------------------------------------------------------------------    
    # battery cell properties
    # --------------------------------------------------------------------------------- 
    As_cell                   = battery_module.cell.surface_area
    cell_mass                 = battery_module.cell.mass     
    #battery_module_data       = battery_module.cell.discharge_performance_map
    
    # ---------------------------------------------------------------------------------
    # Compute Bus electrical properties 
    # ---------------------------------------------------------------------------------    
    bus_conditions              = state.conditions.energy[bus.tag] #what does .tag mean
    bus_config                  = bus.battery_module_electric_configuration
    E_bus                       = bus_conditions.energy
    P_bus                       = bus_conditions.power_draw
    I_bus                       = bus_conditions.current_draw
    
    # ---------------------------------------------------------------------------------
    # Compute battery_module Conditions
    # -------------------------------------------------------------------------    
    battery_module_conditions = state.conditions.energy[bus.tag].battery_modules[battery_module.tag]  #understand this, need to make sure data being fed in is updated
    
    #E_module_max       = battery_module.maximum_energy * battery_module_conditions.cell.capacity_fade_factor #capacity 
    E_module_max       = battery_module.maximum_energy #need to update maximum energy 
    P_module           = battery_module_conditions.power
    P_cell             = battery_module_conditions.cell.power 
    R_0_cell           = battery_module_conditions.cell.internal_resistance
    V_oc_module        = battery_module_conditions.voltage_open_circuit
    V_oc_cell          = battery_module_conditions.cell.voltage_open_circuit  
    #Q_heat_cell        = battery_module_conditions.cell.heat_energy_generated 
    V_ul_cell          = battery_module_conditions.cell.voltage_under_load
    V_ul_module        = battery_module_conditions.voltage_under_load
    
    LLI_cell           = battery_module_conditions.cell.loss_of_lithium_inventory
    Q_heat_cell        = battery_module_conditions.cell.total_heat_generation
    Q_heat_module      = battery_module_conditions.heat_energy_generated

    Cap_lost_cell      = battery_module_conditions.cell.capacity_lost
    pybamm_condition   = battery_module_conditions.cell.pybamm_condition['timestep_' + str(t_idx)] #.solution



    
    I_module           = battery_module_conditions.current 
    I_cell             = battery_module_conditions.cell.current 
    
    SOC_cell           = battery_module_conditions.cell.state_of_charge  
    SOC_module         = battery_module_conditions.state_of_charge
    E_cell             = battery_module_conditions.cell.energy   
    E_module           = battery_module_conditions.energy #mid segment capacity
    # Q_cell             = battery_module_conditions.cell.charge_throughput              
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
    #good

    # ---------------------------------------------------------------------------------------------------
    # Current State 
    # ---------------------------------------------------------------------------------------------------
    if bus_config == 'Series':
        I_module[t_idx]      = I_bus[t_idx]
    elif bus_config  == 'Parallel':
        I_module[t_idx]      = I_bus[t_idx] /len(bus.battery_modules)

    I_cell[t_idx] = I_module[t_idx] / n_parallel   
    Q_heat_module[t_idx]  = Q_heat_cell[t_idx]*n_total    
    P_module[t_idx]       = P_bus[t_idx] /no_modules  - np.abs(Q_heat_module[t_idx]) 

    V_oc_module[t_idx]     = V_oc_cell[t_idx]*n_series 
    V_ul_module[t_idx]     = V_ul_cell[t_idx]*n_series  
    #T_module[t_idx]        = T_cell[t_idx]   # Assume the cell temperature is the temperature of the module
    P_cell[t_idx]          = P_module[t_idx]/n_total 
    E_module[t_idx]        = E_bus[t_idx]/no_modules 
    E_cell[t_idx]          = E_module[t_idx]/n_total 

    # ---------------------------------------------------------------------------------
    # PyBaMM analysis for a single cell
    # --------------------------------------------------------------------------------- 
   
    #model =  battery_module_conditions.cell.pybamm_conditions['timestep_' + str(t_idx)]['model']
    model = battery_module.cell.model.set_initial_conditions_from(pybamm_condition, inplace = False)
    if I_cell[t_idx][0] < 0:
        experiment = pybamm.Experiment( ["Discharge at "+ str(abs(I_cell[t_idx][0])) + " A for " + str(delta_t[t_idx]) + " seconds"] )
        simulation = pybamm.Simulation(model, experiment = experiment, parameter_values = battery_module.cell.battery_parameters) 
        pybamm_condition = simulation.solve(initial_soc = SOC_cell[0][0]) #be careful with [0] may be t_idx
    else:
        experiment = pybamm.Experiment( ["Charge at "+ str(abs(I_cell[t_idx][0])) + " A for " + str(delta_t[t_idx-1]) + " seconds"] )
        simulation = pybamm.Simulation(model, experiment = experiment, parameter_values = battery_module.cell.battery_parameters) 
        pybamm_condition = simulation.solve(initial_soc = SOC_cell[0][0]) #be careful with [0] may be t_idx
    if(t_idx < len(delta_t)):
        E_module[t_idx+1]                                     = (E_module[t_idx]) -P_module[t_idx]*delta_t[t_idx]
        E_module[t_idx+1][E_module[t_idx+1] > E_module_max]   = np.float32(E_module_max)
        SOC_cell[t_idx+1]                                     = E_module[t_idx+1]/E_module_max 

        V_ul_cell[t_idx+1]          = pybamm_condition['Voltage [V]'].data[-1] 
        #I_cell[t_idx+1] = pybamm_condition['Current [A]'].data[-1] 
        R_0_cell[t_idx+1]           = pybamm_condition['Resistance [Ohm]'].data[-1] 
        LLI_cell[t_idx+1]           = pybamm_condition['Loss of lithium inventory [%]'].data[-1]
        Q_heat_cell[t_idx+1]        = pybamm_condition['Total heating [W.m-3]'].data[-1] 
        Cap_lost_cell[t_idx+1]      = pybamm_condition['Total capacity lost to side reactions [A.h]'].data[-1]
        battery_module_conditions.cell.pybamm_condition['timestep_' + str(t_idx+1)] = pybamm_condition

    

    #pybamm_condition = battery_module_conditions.cell.pybamm_state['timestep_' + str(t_idx+1)]['solution'] #how to carry over the solution from the previous time step
    #model = battery_module_conditions.cell.pybamm_state['timestep_' + str(t_idx+1)]['model']
  

    
    stored_results_flag     = True
    stored_battery_module_tag     = battery_module.tag  
        
    return stored_results_flag, stored_battery_module_tag


def reuse_stored_p30b_cell_data(battery_module,state,bus,stored_results_flag, stored_battery_module_tag):
    '''Reuses results from one propulsor for identical batteries
    
    Assumptions: 
    N/A

    Source:
    N/A

    Inputs:  
    

    Outputs:  
    
    Properties Used: 
    N.A.        
    ''' #should not have to modify
   
    state.conditions.energy[bus.tag].battery_modules[battery_module.tag] = deepcopy(state.conditions.energy[bus.tag].battery_modules[stored_battery_module_tag])
    
    return
 