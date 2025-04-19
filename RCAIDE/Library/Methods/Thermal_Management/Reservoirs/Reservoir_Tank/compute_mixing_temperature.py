# RCAIDE/Library/Methods/Thermal_Management/Reservoirs/Reservoir_Tank/compute_mixing_temperature.py


# Created:  Apr 2024, S. Shekar 

# ---------------------------------------------------------------------------------------------------------------------- 
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import  RCAIDE
import numpy as np
from scipy.optimize import fsolve


def compute_mixing_temperature(reservoir, state, coolant_line, delta_t, t_idx):
    """
    Computes the resultant temperature of the reservoir at each time step with coolant exchanging heat to the environment.

    :param reservoir: Reservoir Data Structure
        - reservoir.surface_area
        - reservoir.volume
        - reservoir.thickness
        - reservoir.material.conductivity
        - reservoir.material.emissivity
        - reservoir.coolant
    :type reservoir: dict
    :param state: State Data Structure
        - state.conditions.freestream.temperature
        - state.conditions.energy.coolant_line[reservoir.tag].coolant_temperature
    :type state: dict
    :param coolant_line: Coolant Line Data Structure
    :type coolant_line: dict
    :param delta_t: Time step
    :type delta_t: float
    :param t_idx: Time index
    :type t_idx: int
    :return: Updated temperature of the reservoir coolant
    :rtype: float

    :Assumptions: 
        N/A

    :Source:
        None
    """  
    T_current = 0
    volume    = 0
    for reservoir in coolant_line.reservoirs:
        T_current += state.conditions.energy[coolant_line.tag][reservoir.tag].coolant_temperature[t_idx, 0]
        volume    += reservoir.volume
    
    T_current = T_current / len(coolant_line.reservoirs)

    # Reservoir Properties
    coolant = reservoir.coolant
    rho_coolant = coolant.compute_density(T_current)
    Cp_RES = coolant.compute_cp(T_current)
    mass_coolant  = rho_coolant * volume

    mass_flow_HAS  = []
    T_outlet_HAS   = []
    Cp_HAS         = []
    mass_flow_HEX  = []
    T_outlet_HEX   = []
    Cp_HEX         = []

    for battery in coolant_line.battery_modules:
        for HAS in battery:
            if isinstance(HAS, RCAIDE.Library.Components.Thermal_Management.Batteries.Liquid_Cooled_Wavy_Channel):
                mass_flow_HAS.append(state.conditions.energy[coolant_line.tag][HAS.tag].coolant_mass_flow_rate[t_idx + 1])
                T_outlet_HAS.append(state.conditions.energy[coolant_line.tag][HAS.tag].outlet_coolant_temperature[t_idx + 1])
                Cp_HAS.append(coolant.compute_cp(T_outlet_HAS[-1]))

    for HEX in coolant_line.heat_exchangers:
        mass_flow_HEX.append(state.conditions.energy[coolant_line.tag][HEX.tag].coolant_mass_flow_rate[t_idx + 1])
        T_outlet_HEX.append(state.conditions.energy[coolant_line.tag][HEX.tag].outlet_coolant_temperature[t_idx + 1])
        Cp_HEX.append(coolant.compute_cp(T_outlet_HEX[-1]))

    # Solve for T_final using fsolve
        T_final = fsolve(energy_balance, T_current, args=(T_current, delta_t, mass_coolant, Cp_RES, Cp_HAS, Cp_HEX, mass_flow_HAS, T_outlet_HAS, mass_flow_HEX, T_outlet_HEX, reservoir, state, t_idx))[0]

    # Update the reservoir temperature
    state.conditions.energy[coolant_line.tag][reservoir.tag].coolant_temperature[t_idx + 1, 0] = T_final
    return

def compute_heat_loss_to_environment(T_final, T_ambient, reservoir):
    # Properties of Reservoir
    A_surface       = reservoir.surface_area
    thickness       = reservoir.thickness
    conductivity    = reservoir.material.conductivity
    emissivity_res  = reservoir.material.emissivity

    # Heat Transfer properties
    sigma           = 5.69e-8  # Stefan Boltzmann Constant
    h               = 1000  # [W/m^2-K]
    emissivity_air  = 0.9

    # Heat Transfer due to conduction
    dQ_dt_cond = conductivity * A_surface * (T_final - T_ambient) / thickness

    # Heat Transfer due to natural convection
    dQ_dt_conv = h * A_surface * (T_final - T_ambient)

    # Heat Transfer due to radiation
    dQ_dt_rad = sigma * A_surface * ((emissivity_res * T_final ** 4) - (emissivity_air * T_ambient ** 4))

    return dQ_dt_cond + dQ_dt_conv + dQ_dt_rad

def energy_balance(T_final, T_current, delta_t, mass_coolant, Cp_RES, Cp_HAS, Cp_HEX, mass_flow_HAS, T_outlet_HAS, mass_flow_HEX, T_outlet_HEX, reservoir, state, t_idx):
    # Ambient Air Temperature
    T_ambient = state.conditions.freestream.temperature[t_idx, 0]

    # Compute heat loss to the environment
    dQ_dt_env = compute_heat_loss_to_environment(T_final, T_ambient, reservoir)

    return (T_final - T_current
            - (delta_t / (mass_coolant * Cp_RES)) *
            (sum(mass_flow_HAS) * np.average(Cp_HAS) * (np.average(T_outlet_HAS) - T_final) +
             sum(mass_flow_HEX) * np.average(Cp_HEX) * (np.average(T_outlet_HEX) - T_final) -
             dQ_dt_env))