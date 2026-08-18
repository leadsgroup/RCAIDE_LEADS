# RCAIDE/Methods/Powertrain/Sources/Batteries/Lithium_Ion_LFP/compute_lfp_cell_performance.py
#
#
# Created: Nov 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units
import numpy as np
from copy import deepcopy

# ----------------------------------------------------------------------------------------------------------------------
# compute_lfp_cell_performance
# ----------------------------------------------------------------------------------------------------------------------
def compute_lfp_cell_performance(battery_module, battery, state, network):
    """
    Computes the performance of lithium iron phosphate (LFP) battery cells.

    Cell temperature and state of charge are solved implicitly: this function computes
    their instantaneous rates of change from the current unknown-vector guess (covering
    every control point at once) and packs those into residuals against a derivative
    matrix, exactly like the other battery chemistries. It does not time-march.

    Parameters
    ----------
    battery_module : BatteryModule
        The battery module containing LFP cells
    battery : Battery_Pack
        The battery pack containing this module
    state : State
        Current system state containing conditions for all components
    network : Network
        The powertrain network (used to look up coolant distributors)

    Returns
    -------
    stored_results_flag : bool
        Flag indicating if results were stored
    stored_battery_tag : str
        Tag of the battery module for which results were stored
    """
    # ---------------------------------------------------------------------------------
    # Time discretization
    # ---------------------------------------------------------------------------------
    D = state.numerics.time.differentiate

    # ---------------------------------------------------------------------------------
    # battery cell properties
    # ---------------------------------------------------------------------------------
    electrode_area            = battery_module.cell.electrode_area
    As_cell                   = battery_module.cell.surface_area
    cell_mass                 = battery_module.cell.mass
    Cp                        = battery_module.cell.specific_heat_capacity
    battery_module_data       = battery_module.cell.discharge_performance_map

    # ---------------------------------------------------------------------------------
    # Compute battery_module Conditions
    # -------------------------------------------------------------------------
    battery_module_conditions = state.conditions.energy.sources[battery.tag][battery_module.tag]

    E_module_max       = battery_module.maximum_energy * battery_module_conditions.cell.capacity_fade_factor
    I_module           = battery_module_conditions.current_draw

    # ---------------------------------------------------------------------------------
    # set unknowns
    # ---------------------------------------------------------------------------------
    T_cell_unkn   = state.unknowns.network[battery.tag + '_' + battery_module.tag + '_cell_temperature']
    SOC_cell_unkn = state.unknowns.network[battery.tag + '_' + battery_module.tag + '_cell_state_of_charge']

    # ---------------------------------------------------------------------------------
    # Electrical configuration
    # ---------------------------------------------------------------------------------
    n_series   = battery_module.electrical_configuration.series
    n_parallel = battery_module.electrical_configuration.parallel
    n_total    = n_series * n_parallel

    # Scaling factors for numerical conditioning
    T_scale = 310.0
    E_scale = E_module_max

    T_cell_scaled = T_cell_unkn / T_scale
    SOC_bounded   = np.clip(SOC_cell_unkn, 1e-4, 1.0)

    # ---------------------------------------------------------------------------------
    # Thermal Management System via assigned_distributors
    # ---------------------------------------------------------------------------------
    coolant_line = None
    if battery_module.assigned_distributors is not None:
        for distributor_tag in battery_module.assigned_distributors[0]:
            distributor = network.distributors[distributor_tag]
            if isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Coolant_Line):
                coolant_line = distributor

    # ---------------------------------------------------------------------------------
    # Current, heat generation
    # ---------------------------------------------------------------------------------
    I_cell = I_module / n_parallel

    sigma         = 130
    i_cell        = I_cell / electrode_area
    q_dot_entropy = (4.6810*SOC_bounded**4 - 8.3729*SOC_bounded**3 + 3.7197*SOC_bounded**2 + 0.4356*SOC_bounded - 0.3027)
    q_dot_joule   = (i_cell**2)/sigma
    Q_heat_cell   = (q_dot_joule + q_dot_entropy)*As_cell
    Q_heat_module = Q_heat_cell*n_total

    # ---------------------------------------------------------------------------------
    # Voltage
    # ---------------------------------------------------------------------------------
    T_cell_bounded = np.clip(T_cell_unkn, 263.15, 333.15)
    V_ul_cell       = compute_lfp_cell_state(battery_module, battery_module_data, SOC_bounded.copy(), T_cell_bounded.copy(), np.abs(I_cell).copy())
    V_ul_module     = V_ul_cell*n_series

    # Power draw including the heat generated (mirrors the pre-existing LFP power model)
    P_module = battery_module_conditions.power_draw + np.abs(Q_heat_module)
    P_cell   = P_module/n_total

    # ---------------------------------------------------------------------------------
    # Cell temperature residual
    # ---------------------------------------------------------------------------------
    if coolant_line != None and battery_module.heat_acquisition_system != None:
        HAS   = battery_module.heat_acquisition_system
        dT_dt, Q_to_coolant = HAS.compute_thermal_performance(battery_module,coolant_line,Q_heat_cell,T_cell_bounded,state)
        battery_module_conditions.heat_to_coolant = Q_to_coolant
    else:
        dT_dt = Q_heat_cell/(cell_mass*Cp)
    dT_dt_scaled = dT_dt/T_scale
    R_temp       = np.dot(D, T_cell_scaled)[:, 0] - dT_dt_scaled[:, 0]
    R_temp[0]    = T_cell_scaled[0,0] - battery_module_conditions.cell.temperature[0, 0]/T_scale
    state.residuals.network[battery.tag + '_' + battery_module.tag + '_cell_temperature'] = R_temp

    # ---------------------------------------------------------------------------------
    # State of charge residual
    # ---------------------------------------------------------------------------------
    dE_dt    = -P_module
    R_soc    = np.dot(D, SOC_cell_unkn*E_scale)[:, 0] - dE_dt[:, 0]
    R_soc[0] = SOC_cell_unkn[0,0] - battery_module_conditions.cell.state_of_charge[0, 0]
    state.residuals.network[battery.tag + '_' + battery_module.tag + '_cell_state_of_charge'] = R_soc

    # ---------------------------------------------------------------------------------
    # Charge throughput
    # ---------------------------------------------------------------------------------
    dt      = np.diff(state.numerics.time.control_points[:,0])
    avg_I   = (I_cell[:-1, 0] + I_cell[1:, 0]) / 2
    Q_Ah    = np.atleast_2d(np.concatenate(([0.0], np.cumsum(dt*avg_I)))).T / Units.hr + battery_module_conditions.cell.charge_throughput[0]

    # ---------------------------------------------------------------------------------
    # Store results
    # ---------------------------------------------------------------------------------
    # Module
    battery_module_conditions.current                    = I_module
    battery_module_conditions.heat_energy_generated       = Q_heat_module
    battery_module_conditions.voltage_under_load          = V_ul_module
    battery_module_conditions.state_of_charge[1:,0]       = SOC_cell_unkn[1:,0]
    battery_module_conditions.temperature[1:,0]           = T_cell_unkn[1:,0]
    battery_module_conditions.energy[1:,0]                = SOC_cell_unkn[1:,0] * E_module_max
    battery_module_conditions.depth_of_discharge          = 1. - battery_module_conditions.state_of_charge
    battery_module_conditions.outputs.power.electrical    = P_module

    # Cell
    battery_module_conditions.cell.voltage_under_load     = V_ul_cell
    battery_module_conditions.cell.power                  = P_cell
    battery_module_conditions.cell.current                = I_cell
    battery_module_conditions.cell.heat_energy_generated  = Q_heat_cell
    battery_module_conditions.cell.state_of_charge        = battery_module_conditions.state_of_charge
    battery_module_conditions.cell.temperature            = battery_module_conditions.temperature
    battery_module_conditions.cell.depth_of_discharge     = battery_module_conditions.depth_of_discharge
    battery_module_conditions.cell.energy                 = battery_module_conditions.energy / n_total
    battery_module_conditions.cell.charge_throughput      = Q_Ah

    stored_results_flag = True
    stored_source_tag    = battery_module.tag

    return battery_module_conditions.inputs, battery_module_conditions.outputs, stored_results_flag, stored_source_tag

def reuse_stored_lfp_cell_data(battery_module,state,stored_battery_tag,stored_battery_module_tag):
    """Reuses results from one module for identical battery modules"""

    state.conditions.energy.sources[stored_battery_tag][battery_module.tag] = deepcopy(state.conditions.energy.sources[stored_battery_tag][stored_battery_module_tag])
    inputs  = state.conditions.energy.sources[stored_battery_tag][battery_module.tag].inputs
    outputs = state.conditions.energy.sources[stored_battery_tag][battery_module.tag].outputs
    return inputs, outputs


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
    RCAIDE.Library.Components.Powertrain.Sources.Batteries.Modules.Lithium_Ion_LFP
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
