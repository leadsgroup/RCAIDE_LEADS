# RCAIDE/Library/Methods/Powertrain/Systems/compute_ice_protection_power_draw.py
#
# Created:  May 2026, M. Clarke, S. Sharma

import numpy as np

def compute_ice_protection_power_draw(ice_protection, state, vehicle):
    """Computes the power draw of an ice protection system.

    Combines electro-thermal anti-icing (wing leading edges) and
    electro-mechanical de-icing (engine cowls). Activates only when
    freestream temperature is between 0 C and -30 C.

    Parameters
    ----------
    ice_protection : Ice_Protection
        Ice protection component.
    state : State
        Mission segment state containing freestream conditions.
    vehicle : Vehicle
        The aircraft vehicle (used for wing reference areas).

    Returns
    -------
    inputs : Conditions
        Input power conditions.
    outputs : Conditions
        Output power conditions.
    """
    Area           = 0
    percentage_ice = 0.05
    E_pulse        = 500    # J per pulse
    f_pulse        = 0.2    # pulses per second
    eta_sys        = 0.8
    q_total_flux   = 6000   # W/m^2

    for wing in vehicle.wings:
        Area += wing.areas.reference * percentage_ice

    P_anti_ice     = Area * q_total_flux
    P_de_ice       = (E_pulse * f_pulse) / eta_sys
    P_ice_initial  = P_anti_ice + P_de_ice

    T_ambient  = state.conditions.freestream.temperature
    IPS_active = np.where((T_ambient <= 273.15) & (T_ambient >= 243.15), 1.0, 0.0)
    P_ice      = P_ice_initial * IPS_active

    system_conditions                              = state.conditions.energy.systems[ice_protection.tag]
    system_conditions.inputs.power.electrical[:,0]  = P_ice[:,0]
    system_conditions.outputs.power.electrical[:,0] = 0.0
    return system_conditions.inputs, system_conditions.outputs
