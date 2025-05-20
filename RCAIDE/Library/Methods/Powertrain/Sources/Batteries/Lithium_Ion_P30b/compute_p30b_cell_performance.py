# RCAIDE/Methods/Powertrain/Sources/Batteries/Lithium_Ion_P30b/compute_p30b_cell_performance.py

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
   
    # ---------------------------------------------------------------------------------    
    # battery cell properties
    # --------------------------------------------------------------------------------- 
    As_cell                   = battery_module.cell.surface_area
    cell_mass                 = battery_module.cell.mass     
    
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
    V_ul_cell          = battery_module_conditions.cell.voltage_under_load
    V_ul_module        = battery_module_conditions.voltage_under_load
    
    LLI_cell           = battery_module_conditions.cell.loss_of_lithium_inventory
    Q_heat_cell        = battery_module_conditions.cell.total_heat_generation
    Q_heat_module      = battery_module_conditions.heat_energy_generated
    Discharge_cell     = battery_module_conditions.cell.discharge_capacity
    Max_capacity_cell  = battery_module_conditions.cell.max_capacity

    Cap_lost_cell      = battery_module_conditions.cell.capacity_lost


    Q_cell             = battery_module_conditions.cell.charge_throughput
    T_cell             = battery_module_conditions.cell.temperature
    T_module             = battery_module_conditions.temperature
    
    I_module           = battery_module_conditions.current 
    I_cell             = battery_module_conditions.cell.current 
    
    SOC_cell           = battery_module_conditions.cell.state_of_charge  
    SOC_module         = battery_module_conditions.state_of_charge
    E_cell             = battery_module_conditions.cell.energy   
    E_module           = battery_module_conditions.energy #mid segment capacity
    # Q_cell             = battery_module_conditions.cell.charge_throughput              
    DOD_cell           = battery_module_conditions.cell.depth_of_discharge


    pybamm_condition   = battery_module_conditions.cell.pybamm_condition['timestep_' + str(t_idx)] #.solution

    
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
    P_module[t_idx]       = P_bus[t_idx] /no_modules  - np.abs(Q_heat_module[t_idx]) #units mismatch?
    P_module[t_idx]       = P_bus[t_idx] /no_modules 

    V_oc_module[t_idx]     = V_oc_cell[t_idx]*n_series 
    V_ul_module[t_idx]     = V_ul_cell[t_idx]*n_series  
    T_module[t_idx]        = T_cell[t_idx]
    P_cell[t_idx]          = P_module[t_idx]/n_total 
    E_module[t_idx]        = E_bus[t_idx]/no_modules 
    E_cell[t_idx]          = E_module[t_idx]/n_total 

    # ---------------------------------------------------------------------------------
    # PyBaMM analysis for a single cell
    # --------------------------------------------------------------------------------- 
   
    model = battery_module.cell.model.set_initial_conditions_from(pybamm_condition, inplace = True)
    params = battery_module.cell.battery_parameters
    params.update({'Initial temperature [K]': T_cell[t_idx][0]})

 
    #over charge prevention using cccv, need to go back to all previous states

    # if SOC_cell[t_idx] > 1:
    #     SOC_cell[t_idx] = 1
    #     pybamm_condition  = battery_module_conditions.cell.pybamm_condition['timestep_' + str(t_idx-1)] #set the state to the previous state
    #     experiment = pybamm.Experiment( ["Charge at " + str(abs(P_cell[t_idx][0])) + " W for " + str(delta_t[t_idx-1]) + " seconds"] )
    #     simulation = pybamm.Simulation(model, experiment = experiment, parameter_values = params, solver = pybamm.IDAKLUSolver()) 
    #     pybamm_condition = simulation.solve() 

    if P_bus[t_idx][0] > 0: #power is positive when discharging
                experiment = pybamm.Experiment( ["Discharge at "+ str(abs(I_cell[t_idx][0])) + " A for " + str(delta_t[t_idx-1]) + " seconds"] )
                simulation = pybamm.Simulation(model, experiment = experiment, parameter_values = params, solver = pybamm.IDAKLUSolver()) 
                pybamm_condition = simulation.solve()
    else:
                if pybamm_condition['Voltage [V]'].data[-1] >=  4.2:
                    experiment = pybamm.Experiment( ["Hold at 4.2V for " + str(delta_t[t_idx-1]) + " seconds or until 0.1 A"])
                    simulation = pybamm.Simulation(model, experiment = experiment, parameter_values = params, solver = pybamm.IDAKLUSolver()) 
                    pybamm_condition = simulation.solve() 
                else: 
                    experiment = pybamm.Experiment( ["Charge at " + str(abs(P_cell[t_idx][0])) + " W for " + str(delta_t[t_idx-1]) + " second or until 4.2 V"] )
                    simulation = pybamm.Simulation(model, experiment = experiment, parameter_values = params, solver = pybamm.IDAKLUSolver()) 
                    pybamm_condition = simulation.solve() 
    if(t_idx < len(delta_t)):
        Discharge_cell[t_idx+1]     = pybamm_condition['Discharge capacity [A.h]'].data[-1] #cumulative
        Max_capacity_cell[t_idx+1]  = pybamm_condition.summary_variables['Capacity [A.h]'][-1]
        SOC_cell[t_idx+1]           = SOC_cell[t_idx] - ((Discharge_cell[t_idx+1]-Discharge_cell[t_idx])/Max_capacity_cell[t_idx])

        V_ul_cell[t_idx+1]          = pybamm_condition['Voltage [V]'].data[-1] 

        E_module[t_idx+1]                                     = (E_module[t_idx]) - P_module[t_idx]*delta_t[t_idx]
        E_module[t_idx+1][E_module[t_idx+1] > E_module_max]   = np.float32(E_module_max)
        Q_cell[t_idx+1]                                       = Q_cell[t_idx] + abs(I_cell[t_idx])*delta_t[t_idx]/Units.hr
        R_0_cell[t_idx+1]           = pybamm_condition['Resistance [Ohm]'].data[-1] 
        LLI_cell[t_idx+1]           = pybamm_condition['Loss of lithium inventory [%]'].data[-1]
        T_cell[t_idx+1]             = pybamm_condition['Cell temperature [K]'].data[-1][-1]
        Q_heat_cell[t_idx+1]        = pybamm_condition['Total heating [W.m-3]'].data[-1][-1] 
        Cap_lost_cell[t_idx+1]      = pybamm_condition['Total capacity lost to side reactions [A.h]'].data[-1]
      

        battery_module_conditions.cell.pybamm_condition['timestep_' + str(t_idx+1)] = pybamm_condition




        # Compute cell temperature
        if HAS is not None:
            T_cell[t_idx+1]  = HAS.compute_thermal_performance(battery_module,bus,coolant_line,Q_heat_cell[t_idx],T_cell[t_idx],state,delta_t[t_idx],t_idx)
        else:
            T_cell[t_idx+1]    = pybamm_condition['Cell temperature [K]'].data[-1][-1]
  
    stored_results_flag     = True
    stored_battery_module_tag     = battery_module.tag  
        
    return stored_results_flag, stored_battery_module_tag


def reuse_stored_p30b_cell_data(battery_module,state,bus,stored_results_flag, stored_battery_module_tag):
   
    state.conditions.energy[bus.tag].battery_modules[battery_module.tag] = deepcopy(state.conditions.energy[bus.tag].battery_modules[stored_battery_module_tag])
    
    return
 