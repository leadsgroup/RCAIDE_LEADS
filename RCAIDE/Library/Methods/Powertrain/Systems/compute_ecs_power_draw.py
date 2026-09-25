# RCAIDE/Library/Methods/Powertrain/Systems/compute_ecs_power_draw.py
#
# Created:  May 2026, M. Clarke, S. Sharma

import numpy as np

def compute_number_of_windows(vehicle):
    """Estimates total window count from the cabin layout: one window per row,
    per side (left + right), summed across every cabin class in every cabin in
    every fuselage. Falls back to 0 if no cabin layout is defined, matching
    the previous hardcoded default.

    Assumptions:
    One window per row per side -- a common single-aisle/narrow-body pattern,
    not universally true (e.g. rows can share a window, or a window can be
    blocked out over a wing/door), so this is an estimate, not a geometric fact.
    """
    total_rows = 0
    for fuselage in vehicle.fuselages:
        for cabin in fuselage.cabins:
            for cabin_class in cabin.classes:
                total_rows += cabin_class.number_of_rows
    return total_rows * 2


def compute_ecs_power_draw(environmental_controls, state, vehicle):
    """Computes the power draw of an environmental control system.

    Dynamically evaluates cabin air compressor power based on altitude and
    Mach number, recirculation fan power, plus steady-state vapor cycle
    cooling from passenger/system heat.

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
    m_dot_per_pax = 0.00416   # kg/s per passenger, CS 25.831 minimum fresh air
    Q_per_pax     = 70        # W per passenger
    Q_sys_per_pax = 40        # W (IFE/avionics/galley heat per pax)
    Q_sun         = 1367      # W/m^2 solar constant
    A_window      = 0.08      # m^2 per window
    N_windows     = compute_number_of_windows(vehicle)

    T_cabin       = 297.15    # K, 24 C target cabin temperature
    T_in_min      = 291.15    # K, 18 C, SAE ARP85G lower bound on cabin inlet temperature
    T_in_max      = 302.15    # K, 29 C, SAE ARP85G upper bound
    epsilon       = 0.5       # recirculation rate (fraction of total cabin flow that's recirculated, not fresh)
    eta_fan       = 0.7       # recirculation fan efficiency
    fan_pressure_ratio = 1.1  # p2/p1 across the recirculation fan
    R_air         = 287.05    # J/(kg*K), specific gas constant for air

    altitude = state.conditions.freestream.altitude
    Mach     = state.conditions.freestream.mach_number
    Cp       = state.conditions.freestream.constant_pressure_specific_heat
    T1       = state.conditions.freestream.temperature
    P1       = state.conditions.freestream.pressure
    gamma    = state.conditions.freestream.specific_heat
    eta_c    = environmental_controls.cabin_compressor_efficiency

    # Heat loads and the mass flow actually needed to hold them in range --
    # the bare CS 25.831 floor (m_dot_nom) is often not enough: at typical
    # passenger/system heat loads it can imply a cabin inlet temperature well
    # below the 18-29 C SAE ARP85G band, so the flow has to be increased to
    # whatever holds that floor (or ceiling). Missing this step understates
    # compressor power by several times, since compressor power scales
    # ~linearly with mass flow -- this was the dominant gap, much larger than
    # the fan term added below.
    Q_pax   = N_pax * Q_per_pax
    Q_sys   = N_pax * Q_sys_per_pax
    Q_solar = 0.0  # Q_sun * A_window * N_windows -- disabled pending validation, see compute_ecs_power_draw's notes
    Q_total = Q_pax + Q_sys + Q_solar

    m_dot_nom         = m_dot_per_pax * N_pax
    T_in_uncorrected  = T_cabin - Q_total / (Cp * m_dot_nom)
    m_dot_cabin       = np.where(
        T_in_uncorrected < T_in_min, Q_total / (Cp * (T_cabin - T_in_min)),
        np.where(T_in_uncorrected > T_in_max, Q_total / (Cp * (T_in_max - T_cabin)), m_dot_nom))

    # Only the fresh-air fraction of the (possibly corrected) cabin flow
    # passes through the packs/compressor -- the rest is recirculated -- and
    # that's floored at the regulatory minimum regardless.
    m_dot_fresh  = (1 - epsilon) * m_dot_cabin
    m_dot_packs  = np.maximum(m_dot_fresh, m_dot_nom)
    m_dot_recirc = epsilon * m_dot_cabin

    T_ram   = T1 * (1 + ((gamma - 1) / 2) * Mach**2)
    P_ram   = P1 * (1 + ((gamma - 1) / 2) * Mach**2)
    P_cabin = np.where(altitude < 2438.4, P_ram + 20000, 78000)
    # Compression starts from the ram-recovered state (T_ram, P_ram), not raw
    # freestream (T1, P1) -- the ram effect already does part of the
    # compression "for free" using the aircraft's own kinetic energy; only the
    # remaining rise from ram state to cabin pressure costs compressor power.
    P_comp  = (m_dot_packs * Cp * T_ram / eta_c) * ((P_cabin / P_ram)**((gamma - 1) / gamma) - 1)

    # Recirculation fan: isentropic work to move the recirculated fraction
    # back through the mixing unit, at the cabin's own temperature/pressure
    # rise (not the freestream ram state above -- this air never left the
    # cabin).
    w_fan  = (gamma / (gamma - 1)) * R_air * T_cabin * (fan_pressure_ratio**((gamma - 1) / gamma) - 1)
    P_fan  = w_fan * m_dot_recirc / eta_fan

    # No separate vapor-cycle cooling term: the paper's model never applies a
    # COP to cabin heat loads -- the conditioned/compressed air itself is the
    # cooling mechanism, and that's already accounted for by m_dot_cabin's
    # heat-balance correction above. A separate P_cool = Q_total/COP term
    # double-counts the same heat removal a second time.
    P_ecs   = P_comp + P_fan

    system_conditions                              = state.conditions.energy.systems[environmental_controls.tag]
    system_conditions.inputs.power.electrical[:,0]  = P_ecs[:,0]
    system_conditions.outputs.power.electrical[:,0] = 0.0
    return system_conditions.inputs, system_conditions.outputs
