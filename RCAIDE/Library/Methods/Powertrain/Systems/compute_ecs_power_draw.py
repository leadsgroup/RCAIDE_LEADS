# RCAIDE/Library/Methods/Powertrain/Systems/compute_ecs_power_draw.py
#
# Created:  May 2026, M. Clarke, S. Sharma

import numpy as np

def compute_ecs_power_draw(environmental_controls, state, vehicle):
    """Computes the power draw of an environmental control system.

    Dynamically evaluates cabin air compressor power based on altitude and
    Mach number, plus steady-state vapor cycle cooling from passenger/system heat.

    Parameters
    ----------
    environmental_controls : Environmental_Controls
        ECS component with cabin_compressor_efficiency attribute.
    state : State
        Mission segment state containing freestream conditions.
    vehicle : Vehicle
        The aircraft vehicle (used for passenger count).

    Returns
    -------
    inputs : Conditions
        Input power conditions.
    outputs : Conditions
        Output power conditions.
    """
    N_pax         = vehicle.number_of_passengers
    m_dot_per_pax = 0.00416   # kg/s per passenger
    Q_per_pax     = 70        # W per passenger
    Q_sys_per_pax = 40        # W (IFE/avionics/galley heat per pax)
    COP           = 2.5       # vapor cycle coefficient of performance
    Q_sun         = 1367      # W/m^2 solar constant
    A_window      = 0.08      # m^2 per window
    N_windows     = 0

    # Compressor power (dynamic with altitude/Mach)
    altitude = state.conditions.freestream.altitude
    Mach     = state.conditions.freestream.mach_number
    Cp       = state.conditions.freestream.constant_pressure_specific_heat
    T1       = state.conditions.freestream.temperature
    P1       = state.conditions.freestream.pressure
    gamma    = state.conditions.freestream.specific_heat
    m_dot    = m_dot_per_pax * N_pax
    eta_c    = environmental_controls.cabin_compressor_efficiency

    P_ram   = P1 * (1 + ((gamma - 1) / 2) * Mach**2)
    P_cabin = np.where(altitude < 2438.4, P_ram + 20000, 78000)
    P_comp  = (m_dot * Cp * T1 / eta_c) * ((P_cabin / P1)**((gamma - 1) / gamma) - 1)

    # Vapor cycle cooling (steady-state)
    Q_pax   = N_pax * Q_per_pax
    Q_sys   = N_pax * Q_sys_per_pax
    Q_solar = Q_sun * A_window * N_windows
    P_cool  = (Q_pax + Q_sys + Q_solar) / COP

    P_ecs   = P_comp + P_cool

    system_conditions                              = state.conditions.energy.systems[environmental_controls.tag]
    system_conditions.inputs.power.electrical[:,0]  = P_ecs[:,0]
    system_conditions.outputs.power.electrical[:,0] = 0.0
    return system_conditions.inputs, system_conditions.outputs
