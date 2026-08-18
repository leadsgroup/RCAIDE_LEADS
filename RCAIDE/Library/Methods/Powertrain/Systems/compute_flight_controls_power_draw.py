# RCAIDE/Library/Methods/Powertrain/Systems/compute_flight_controls_power_draw.py
#
# Created:  Jul 2024, RCAIDE Team

def compute_flight_controls_power_draw(flight_controls, state, vehicle):
    """Computes the electrical power draw of the flight controls system.

    Parameters
    ----------
    flight_controls : Flight_Controls
        Flight controls component with power_draw attribute [W].
    state : State
        Mission segment state containing conditions.
    vehicle : Vehicle
        The aircraft vehicle.

    Returns
    -------
    inputs : Conditions
        Input power conditions.
    outputs : Conditions
        Output power conditions.
    """
    system_conditions                              = state.conditions.energy.systems[flight_controls.tag]
    system_conditions.inputs.power.electrical[:,0]  = flight_controls.power_draw
    system_conditions.outputs.power.electrical[:,0] = 0.0
    return system_conditions.inputs, system_conditions.outputs
