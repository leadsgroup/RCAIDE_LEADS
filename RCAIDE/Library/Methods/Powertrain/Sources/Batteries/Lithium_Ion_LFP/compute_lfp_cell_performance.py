# RCAIDE/Methods/Powertrain/Sources/Batteries/Lithium_Ion_LFP/compute_lfp_cell_performance.py
# 
# 
# Created: Nov 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
import numpy as np  
from copy import deepcopy

# ----------------------------------------------------------------------------------------------------------------------
# compute_lfp_cell_performance
# ----------------------------------------------------------------------------------------------------------------------  
def compute_lfp_cell_performance(battery_module, state, bus, coolant_lines, t_idx, delta_t):
    """
    Computes the performance of lithium iron phosphate (LFP) battery cells.
    
    Parameters
    ----------
    battery_module : BatteryModule
        The battery module containing LFP cells with the following attributes:
            - cell.electrode_area : float
                Area of the cell electrode [m²]
            - cell.surface_area : float
                Surface area of the cell [m²]
            - cell.mass : float
                Mass of the cell [kg]
            - cell.specific_heat_capacity : float
                Specific heat capacity of the cell [J/(kg·K)]
            - cell.discharge_performance_map : function
                Function that maps C-rate, temperature, and discharge capacity to voltage
            - electrical_configuration.series : int
                Number of cells in series
            - electrical_configuration.parallel : int
                Number of cells in parallel
    state : State
        Current system state containing conditions for all components
    bus : ElectricalBus
        The electrical bus connected to the battery module
    coolant_lines : list
        List of coolant lines that may be connected to the battery module
    t_idx : int
        Current time index in the simulation
    delta_t : numpy.ndarray
        Time step array [s]
    
    Returns
    -------
    stored_results_flag : bool
        Flag indicating if results were stored
    stored_battery_tag : str
        Tag of the battery module for which results were stored
    
    Notes
    -----
    This function computes the electrical and thermal performance of LFP battery cells
    within a battery module. It calculates:
        1. Current flow through the module and individual cells
        2. Heat generation due to joule heating and entropy changes
        3. Cell voltage under load based on state of charge, temperature, and current
        4. Power flow through the module
        5. Temperature changes based on heat generation and cooling (if present)
        6. State of charge and depth of discharge updates
        7. Charge throughput tracking
    
    **Major Assumptions**
        * All battery cells in a module exhibit the same thermal behavior
        * The cell temperature is assumed to be the temperature of the entire module
    
    **Theory**
    
    Heat generation in the cell includes both joule heating and entropic effects:
    
    .. math::
        \\dot{Q}_{joule} = \\frac{i_{cell}^2}{\sigma}
    
    .. math::
        \\dot{Q}_{entropy} = f(SOC) \\cdot T \\cdot \\frac{dU}{dT}
    
    where:
        - :math:`i_{cell}` is the current density [A/m²]
        - :math:`\\sigma` is the electrical conductivity [S/m]
        - :math:`SOC` is the state of charge
        - :math:`T` is the temperature [K]
        - :math:`\\frac{dU}{dT}` is the entropic coefficient
    
    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_LFP
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
    bus_conditions              = state.conditions.energy.busses[bus.tag]
    bus_config                  = bus.battery_module_electric_configuration 
    psi                         = state.conditions.energy.battery_fuel_cell_power_split_ratio
    E_bus                       = bus_conditions.energy
    P_bus                       = bus_conditions.power_draw*psi
    I_bus                       = bus_conditions.current_draw*psi   
    
    # ---------------------------------------------------------------------------------
    # Compute battery_module Conditions
    # -------------------------------------------------------------------------    
    battery_module_conditions = state.conditions.energy.busses[bus.tag].battery_modules[battery_module.tag]  
   
    E_module_max       = battery_module.maximum_energy * battery_module_conditions.cell.capacity_fade_factor
    
    V_oc_module        = battery_module_conditions.voltage_open_circuit
    V_oc_cell          = battery_module_conditions.cell.voltage_open_circuit   
  
    P_module           = battery_module_conditions.power
    P_cell             = battery_module_conditions.cell.power
    
    
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
    n_total           = n_series * n_parallel
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
        I_module[t_idx]      = I_bus[t_idx] / len(bus.battery_modules)

    I_cell[t_idx] = I_module[t_idx] / n_parallel   
       
    # ---------------------------------------------------------------------------------
    # Compute battery_module cell temperature 
    # ---------------------------------------------------------------------------------
    # Determine temperature increase         
    sigma                 =  130  
    i_cell                = I_cell[t_idx]/electrode_area # current intensity (A/m²)
    q_dot_entropy         = (4.6810 * SOC_cell[t_idx]**4 + (-8.3729) * SOC_cell[t_idx]**3 + 3.7197 * SOC_cell[t_idx]**2 + 0.4356 * SOC_cell[t_idx]+ (-0.3027)) # Obtained from curve fitting the dUdt curve  
    q_dot_joule           = (i_cell**2)/(sigma)          
    Q_heat_cell[t_idx]    = (q_dot_joule + q_dot_entropy)*As_cell 
    Q_heat_module[t_idx]  = Q_heat_cell[t_idx]*n_total  
    V_ul_cell[t_idx]      = compute_lfp_cell_state(battery_module,battery_module_data,SOC_cell[t_idx],T_cell[t_idx],abs(I_cell[t_idx])) 
 
    # Effective Power flowing through battery_module 
    P_module[t_idx]       = P_bus[t_idx] /no_modules + np.abs(Q_heat_module[t_idx]) 

    # store remaining variables  
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
            T_cell[t_idx+1]    = T_cell[t_idx] + dT_dt*delta_t[t_idx]
            
        # Compute state of charge and depth of discarge of the battery_module
        E_module[t_idx+1]                                     = np.float32(E_module[t_idx] -P_module[t_idx]*delta_t[t_idx])
        E_module[t_idx+1][E_module[t_idx+1] > E_module_max]   = np.float32(E_module_max)
        SOC_cell[t_idx+1]                                     = E_module[t_idx+1]/E_module_max 
        SOC_cell[t_idx+1][SOC_cell[t_idx+1]>1]                = 1.
        SOC_cell[t_idx+1][SOC_cell[t_idx+1]<0]                = 0. 
        DOD_cell[t_idx+1]                                     = 1 - SOC_cell[t_idx+1]  
        SOC_module[t_idx+1]                                   = SOC_cell[t_idx+1]
    
        # Determine new charge throughput (the amount of charge gone through the battery)
        Q_cell[t_idx+1]    = Q_cell[t_idx] + abs(I_cell[t_idx])*delta_t[t_idx]/Units.hr
        
    stored_results_flag     = True
    stored_battery_tag     = battery_module.tag  
        
    return stored_results_flag, stored_battery_tag
def reuse_stored_lfp_cell_data(battery_module,state,bus,stored_results_flag, stored_battery_tag):
    """Reuses results from one propulsor for identical batteries       
    """
   
    state.conditions.energy.busses[bus.tag].battery_modules[battery_module.tag] = deepcopy(state.conditions.energy.busses[bus.tag].battery_modules[stored_battery_tag])      
    return


def compute_lfp_cell_state(battery_module, battery_module_data, SOC, T, I):
    """
    Computes the electrical state variables of a lithium iron phosphate (LFP) battery cell using look-up tables.
    
    Parameters
    ----------
    battery_module : BatteryModule
        The battery module containing LFP cells with the following attributes:
            - cell.nominal_capacity : float
                Nominal capacity of the cell [Ah]
    battery_module_data : function
        Look-up function that maps C-rate, temperature, and discharge capacity to voltage
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
    This function computes the voltage of an LFP battery cell under load conditions
    by using a look-up table approach. It converts the state of charge to discharge
    capacity, calculates the C-rate, and then uses these values along with temperature
    to determine the cell voltage.
    
    The function applies limits to ensure the inputs are within the valid range of the
    look-up data:
        - SOC is limited to [0, 1]
        - Temperature is limited to [-10°C, 60°C]
        - Current is limited to [0A, 52A]
    
    **Major Assumptions**
        * The model is valid only within the specified temperature and current ranges
    
    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_LFP
    """

    # Make sure things do not break by limiting current, temperature and current 
    capacity      = battery_module.cell.nominal_capacity
    SOC[SOC < 0.]   = 0.  
    SOC[SOC > 1.]   = 1.    
    DOD             = 1 - SOC 
    discharge_capacity = DOD*capacity
    

    T              = T-273
    # Operating Limits of the cell
    T[T<-10]       = -10 # model does not fit for below -10  degrees
    T[T>60]        =  60 # model does not fit for above 60 degrees

    
    I[I<0.0]      = 0.0
    I[I>52.0]     = 52.0
    C_rate        = I/capacity
     

    V_ul  = battery_module_data(C_rate, T, discharge_capacity)
    
    return V_ul