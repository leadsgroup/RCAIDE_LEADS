# RCAIDE/Library/Methods/Powertrain/Systems/compute_hydraulics_power_draw.py
#
# Created:  May 2026, M. Clarke, S. Sharma

def compute_hydraulics_power_draw(hydraulics, state, vehicle):
    """Computes the power draw of a triple-redundant hydraulic system.

    Scales nominal volumetric flow rates linearly with MTOW relative to
    an A320-200 baseline, then computes pump power from pressure difference.

    Parameters
    ----------
    hydraulics : Hydraulics
        Hydraulic system with left, right, and central circuit attributes.
    state : State
        Mission segment state containing conditions.
    vehicle : Vehicle
        The aircraft vehicle (used for MTOW scaling).

    Returns
    -------
    inputs : Conditions
        Input power conditions.
    outputs : Conditions
        Output power conditions.
    """
    MTOW_baseline  = 75166.0  # kg (A320-200 reference)
    P_res          = 3.52     # bar (reservoir pressure)
    eta_pump       = 0.855

    MTOW           = vehicle.mass_properties.max_takeoff
    scaling_factor = MTOW / MTOW_baseline

    V_flow_left    = (hydraulics.left_system.flowspeed    * scaling_factor) / 60000.0
    V_flow_right   = (hydraulics.right_system.flowspeed   * scaling_factor) / 60000.0
    V_flow_central = (hydraulics.central_system.flowspeed * scaling_factor) / 60000.0

    delta_p_left    = (hydraulics.left_system.system_power    - P_res) * 100000.0
    delta_p_right   = (hydraulics.right_system.system_power   - P_res) * 100000.0
    delta_p_central = (hydraulics.central_system.system_power - P_res) * 100000.0

    P_sys_left    = hydraulics.left_system.number_of_pumps    * ((V_flow_left    * delta_p_left   ) / eta_pump)
    P_sys_right   = hydraulics.right_system.number_of_pumps   * ((V_flow_right   * delta_p_right  ) / eta_pump)
    P_sys_central = hydraulics.central_system.number_of_pumps * ((V_flow_central * delta_p_central) / eta_pump)

    P_act = P_sys_left + P_sys_right + P_sys_central

    system_conditions                              = state.conditions.energy.systems[hydraulics.tag]
    system_conditions.inputs.power.electrical[:,0]  = P_act
    system_conditions.outputs.power.electrical[:,0] = 0.0
    return system_conditions.inputs, system_conditions.outputs
