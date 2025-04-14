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

model_base = pybamm.lithium_ion.SPM( #initializing first model to have degradation
        {
            "SEI": "solvent-diffusion limited",
            "SEI porosity change": "true",
            "lithium plating": "partially reversible",
            "lithium plating porosity change": "true",  # alias for "SEI porosity change"
            "particle mechanics": ("swelling and cracking", "swelling only"),
            "SEI on cracks": "true",
            "loss of active material": "stress-driven",
            "calculate discharge energy": "true",  # for compatibility with older PyBaMM versions  
            #"thermal": "lumped"
        }
)
params = pybamm.ParameterValues("Chen2020")
params.update({"Electrode width [m]": 9.06108669e-01,
                "Negative electrode density [kg.m-3]": 2.23367642e+03,
                "Positive electrode density [kg.m-3]": 3.35604961e+03,
                'Bulk solvent concentration [mol.m-3]': 9.04026059e+02,
                'Cation transference number': 1.00108959e+00,
                'Nominal cell capacity [A.h]': 3.43328039e+00,
                'Negative electrode porosity': 9.85780579e-01,
                'Separator porosity':1.01419255e+00, 
                "Ambient temperature [K]": 296.15,
                'Reference temperature [K]': 296.15
                })


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
    As_cell                   = battery_module.cell.surface_area
    cell_mass                 = battery_module.cell.mass    
    Cp                        = battery_module.cell.specific_heat_capacity       
    #battery_module_data       = battery_module.cell.discharge_performance_map
    
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
    battery_module_conditions = state.conditions.energy[bus.tag].battery_modules[battery_module.tag]  #understand this, need to make sure data being fed in is updated
    
    #E_module_max       = battery_module.maximum_energy * battery_module_conditions.cell.capacity_fade_factor #capacity 
    E_module_max       = battery_module_conditions.maximum_energy #need to update maximum energy 
    P_module           = battery_module_conditions.power
    P_cell             = battery_module_conditions.cell.power 
    LLI_cell           = battery_module_conditions.cell.loss_of_lithium_inventory 
    R_0_cell           = battery_module_conditions.cell.internal_resistance 
    Q_heat_cell        = battery_module_conditions.cell.heat_energy_generated 
    V_ul_cell          = battery_module_conditions.cell.voltage_under_load
    
    I_module           = battery_module_conditions.current 
    I_cell             = battery_module_conditions.cell.current 
    
    # SOC_cell           = battery_module_conditions.cell.state_of_charge  
    # SOC_module         = battery_module_conditions.state_of_charge
    # E_cell             = battery_module_conditions.cell.energy   
    # E_module           = battery_module_conditions.energy #mid segment capacity
    # Q_cell             = battery_module_conditions.cell.charge_throughput              
    # DOD_cell           = battery_module_conditions.cell.depth_of_discharge
    
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
       



    # ---------------------------------------------------------------------------------
    # PyBaMM analysis for a single cell
    # --------------------------------------------------------------------------------- 
    if t_idx == 0:
        model = pybamm.lithium_ion.SPM( #initializing first model to have degradation
        {
            "SEI": "solvent-diffusion limited",
            "SEI porosity change": "true",
            "lithium plating": "partially reversible",
            "lithium plating porosity change": "true",  # alias for "SEI porosity change"
            "particle mechanics": ("swelling and cracking", "swelling only"),
            "SEI on cracks": "true",
            "loss of active material": "stress-driven",
            "calculate discharge energy": "true",  # for compatibility with older PyBaMM versions  
            #"thermal": "lumped"
        }
        )
        params = pybamm.ParameterValues("Chen2020")
        params.update({"Electrode width [m]": 9.06108669e-01,
                        "Negative electrode density [kg.m-3]": 2.23367642e+03,
                        "Positive electrode density [kg.m-3]": 3.35604961e+03,
                        'Bulk solvent concentration [mol.m-3]': 9.04026059e+02,
                        'Cation transference number': 1.00108959e+00,
                        'Nominal cell capacity [A.h]': 3.43328039e+00,
                        'Negative electrode porosity': 9.85780579e-01,
                        'Separator porosity':1.01419255e+00, 
                        "Ambient temperature [K]": 296.15,
                        'Reference temperature [K]': 296.15
                        })
        

        experiment = pybamm.Experiment(
            ["Discharge at "+ str(I_cell[t_idx]) + "A for " + str(delta_t) + " seconds"]#might be delta_t[t_idx]
        ) #can change to step format 
        simulation = pybamm.Simulation(model, experiment = experiment, parameter_values = params) #setup simulation
        simulation_solved = simulation.solve(initial_soc = state.SOC) #solving for time segment

        initial_conditions =  deepcopy(simulation_solved)
    else: 
        model = model.set_initial_conditions_from(initial_conditions, inplace = False)

        experiment = pybamm.Experiment(
            ["Discharge at "+ str(I_cell[t_idx]) + "A for " + str(delta_t) + " seconds"]#might be delta_t[t_idx]
        ) #can change to step format 
        simulation = pybamm.Simulation(model, experiment = experiment, parameter_values = params) #setup simulation
        simulation_solved = simulation.solve(initial_soc = state.SOC) #solving for time segment

    batt_capacity = simulation_solved['Discharge capacity [A.h]'].data[-1] #gives the charge throughput
    V_ul_cell[t_idx+1] = simulation_solved['Voltage [V]'].data[-1] #gives the final voltage of the time segment
    I_cell[t_idx+1] = simulation_solved['Current [A]'].data[-1] #gives the final current of the time segment, same as bus current
    R_0_cell[t_idx+1] = simulation_solved['Resistance [Ohm]'].data[-1] #gives the final resistance of the time segment. not to be used for heat, just used pybamm
    Q_heat_cell[t_idx+1]  = simulation_solved['Total heating [W.m-3]'].data[-1] #gives the heat generated during the time segment. 
    LLI_cell[t_idx+1] = simulation_solved['Loss of lithium inventory [mol]'].data[-1] #gives the loss of lithium inventory during the time segment.
    
    
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
    ''' #should not have to moddify
   
    state.conditions.energy[bus.tag].battery_modules[battery_module.tag] = deepcopy(state.conditions.energy[bus.tag].battery_modules[stored_battery_module_tag])
    
    return
 