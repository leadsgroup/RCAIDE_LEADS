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
def compute_nmc_cell_performance(battery_module, state, bus, coolant_lines, t_idx, delta_t):
    """
    Computes the performance of a lithium-nickel-manganese-cobalt-oxide (NMC) battery cell.

    Parameters
    ----------
    battery_module : RCAIDE.Library.Components.Sources.Battery_Modules.Lithium_Ion_NMC
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
                                    - power_draw : numpy.ndarray
                                        Power draw on the bus [W]
                                    - current_draw : numpy.ndarray
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
    psi                         = state.conditions.energy.battery_fuel_cell_power_split_ratio
    E_bus                       = bus_conditions.energy
    P_bus                       = bus_conditions.power_draw*psi
    I_bus                       = bus_conditions.current_draw*psi 
    
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
    P_module[t_idx]       = P_bus[t_idx] /no_modules  + np.abs(Q_heat_module[t_idx]) 

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
    RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC
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