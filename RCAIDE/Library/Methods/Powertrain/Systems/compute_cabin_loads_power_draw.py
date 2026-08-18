# RCAIDE/Library/Methods/Powertrain/Systems/compute_cabin_loads_power_draw.py
#
# Created:  May 2026, M. Clarke, S. Sharma

def compute_cabin_loads_power_draw(cabin_loads, state, vehicle):
    """Computes the electrical power draw of the cabin loads system.

    Scales IFE, galley, and lighting loads linearly with passenger count.

    Parameters
    ----------
    cabin_loads : Cabin_Loads
        Cabin loads component.
    state : State
        Mission segment state containing conditions.
    vehicle : Vehicle
        The aircraft vehicle (used for passenger count).

    Returns
    -------
    inputs : Conditions
        Input power conditions.
    outputs : Conditions
        Output power conditions.
    """
    N_pax              = vehicle.number_of_passengers
    P_ife_per_pax      = 41
    P_galley_per_pax   = 320 * 0.5
    P_lighting_per_pax = 3.2 + 1.4 + 10
    P_cabin            = N_pax * (P_ife_per_pax + P_galley_per_pax + P_lighting_per_pax)

    system_conditions                              = state.conditions.energy.systems[cabin_loads.tag]
    system_conditions.inputs.power.electrical[:,0]  = P_cabin
    system_conditions.outputs.power.electrical[:,0] = 0.0
    return system_conditions.inputs, system_conditions.outputs
