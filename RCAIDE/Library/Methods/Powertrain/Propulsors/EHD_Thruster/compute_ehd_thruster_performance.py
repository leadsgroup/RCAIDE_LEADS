# RCAIDE/Library/Methods/Powertrain/Propulsors/EHD_Thruster/compute_ehd_thruster_performance.py
#
# Created:  Oct 2026, RCAIDE EHD MVP

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Methods.Powertrain.Converters.EHD_Electrode_Array import (compute_relative_air_density,
     compute_peek_inception_field, compute_inception_voltage, compute_dimensionless_current,
     compute_unit_thrust_and_power, compute_collector_drag_coefficient, compute_electrode_drag,
     check_ehd_electrode_array_inputs)
from RCAIDE.Library.Methods.Powertrain.Modulators.High_Voltage_Converter import compute_hvpc_performance

import numpy as np
from copy import deepcopy

# ----------------------------------------------------------------------------------------------------------------------
#  compute_ehd_thruster_performance
# ----------------------------------------------------------------------------------------------------------------------
def compute_ehd_thruster_performance(propulsor, state, center_of_gravity=[[0.0, 0.0, 0.0]]):
    """
    Computes thrust, moment and power of a wire-to-NACA 0010 EHD thruster at every control point
    (spec 3.3 steps 1-10).

    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Powertrain.Propulsors.EHD_Thruster
        Holds electrode_array and high_voltage_converter
    state : RCAIDE.Framework.Mission.Common.State
        Reads conditions.freestream (pressure, temperature, density, velocity, dynamic_viscosity) and
        conditions.energy.propulsors[tag].throttle
    center_of_gravity : list of lists, optional
        [[x, y, z]] [m]

    Returns
    -------
    thrust : numpy.ndarray
        Net thrust vector [N], [N·(T_unit - D_c - D_w), 0, 0] along body +x
    moment : numpy.ndarray
        cross(origin - CG, thrust) [N·m]
    power : numpy.ndarray
        Electrode power P_EHD [W] (the EHD analogue of rotor shaft power)
    power_elec : numpy.ndarray
        Bus demand P_bus = P_EHD / eta_HV [W]
    stored_results_flag : bool
    stored_propulsor_tag : str

    Notes
    -----
    Steps (spec 3.3):
        1. delta = (p/p_0)(T_0/T)
        2. E_i (R19); V_i = measured value if given, else k_Vi·E_i·a·ln(d/a) (R20)
        3. V_a = V_i + throttle·(V_max - V_i), throttle clipped to [0, 1]; flag V_a >= V_spark
        4. V_hat = V_a/V_i; j_hat (R21)
        5-6. T_unit, P_unit, I_unit (spec 2.4 derived equations)
        7. D_c (user c_d or R24 at Re_c = V_inf·c/nu), D_w
        8. T_net = N·(T_unit - D_c - D_w)
        9. P_EHD = N·P_unit; P_bus = P_EHD/eta_HV; heat = P_bus - P_EHD
        10. Store results in conditions.energy (propulsors, converters, modulators)

    **Major Assumptions**
        * Freestream speed is neglected in the electrical model (R3/R16 correction is Stage 2) but kept
          in the drag model
        * Collectors are not also modelled as wings (drag bookkeeping rule)
        * No internal unknowns: throttle maps directly to voltage

    References
    ----------
    [1] Kahol et al., J. Electrostatics 123, 103815 (2023) (R21-R23).
    [2] Peek, Dielectric Phenomena in High Voltage Engineering (1929) (R19).
    [3] Masuyama & Barrett, Proc. R. Soc. A 469, 20120623 (2013) (R1, R2).
    """
    conditions = state.conditions
    array      = propulsor.electrode_array
    hvpc       = propulsor.high_voltage_converter
    check_ehd_electrode_array_inputs(array)

    fs       = conditions.freestream
    results  = conditions.energy.converters[array.tag]
    p_cond   = conditions.energy.propulsors[propulsor.tag]
    throttle = np.clip(p_cond.throttle, 0.0, 1.0)

    # 1. relative air density
    delta = compute_relative_air_density(fs.pressure, fs.temperature)

    # 2. inception
    a   = array.emitter_diameter / 2.0
    E_i = compute_peek_inception_field(a, delta, array.peek_surface_factor)
    if array.inception_voltage_measured is not None:
        V_i = array.inception_voltage_measured * np.ones_like(E_i)
    else:
        V_i = compute_inception_voltage(E_i, a, array.gap, array.k_Vi)

    # 3. throttle -> voltage
    V_a = V_i + throttle * (array.maximum_voltage - V_i)
    if array.sparkover_voltage is not None:
        sparkover_flag = (V_a >= array.sparkover_voltage).astype(float)
    else:
        sparkover_flag = np.zeros_like(V_a)

    # 4. current shape
    V_hat = V_a / V_i
    j_hat = compute_dimensionless_current(V_hat)

    # 5-6. electrical thrust, power and current per unit
    T_unit, P_unit, I_unit = compute_unit_thrust_and_power(V_a, array.gap, array.ion_mobility, j_hat,
                                                           array.unit_spacing, array.span, array.k_T, array.k_P)

    # 7. drag per unit
    V_inf = fs.velocity
    Re_c  = fs.density * V_inf * array.collector_chord / fs.dynamic_viscosity
    if array.collector_drag_coefficient is not None:
        c_d = array.collector_drag_coefficient * np.ones_like(Re_c)
    else:
        c_d = compute_collector_drag_coefficient(Re_c, array.collector_boundary_layer)
    D_c, D_w = compute_electrode_drag(fs.density, V_inf, array.collector_chord, array.emitter_diameter,
                                      array.span, c_d, array.emitter_drag_coefficient)

    # 8. net thrust
    N     = array.number_of_units
    T_net = N * (T_unit - D_c - D_w)

    # 9. power
    P_EHD = N * P_unit
    P_bus = compute_hvpc_performance(hvpc, conditions, P_EHD, V_a)

    # 10. store
    results.relative_air_density       = delta
    results.inception_field            = E_i
    results.inception_voltage          = V_i
    results.applied_voltage            = V_a
    results.normalized_voltage         = V_hat
    results.dimensionless_current      = j_hat
    results.unit_current               = I_unit
    results.unit_thrust                = T_unit
    results.unit_power                 = P_unit
    results.collector_reynolds_number  = Re_c
    results.collector_drag_coefficient = c_d
    results.unit_collector_drag        = D_c
    results.unit_wire_drag             = D_w
    results.sparkover_flag             = sparkover_flag

    thrust      = np.zeros((T_net.shape[0], 3))
    thrust[:,0] = T_net[:,0]
    moment      = _compute_moment(propulsor, thrust, center_of_gravity)

    p_cond.thrust                = thrust
    p_cond.moment                = moment
    p_cond.power                 = P_EHD
    p_cond.net_thrust            = T_net
    p_cond.electrode_power       = P_EHD
    p_cond.bus_power             = P_bus
    p_cond.thrust_to_power_ratio = np.divide(T_unit, P_unit, out=np.zeros_like(T_unit), where=P_unit > 0)

    stored_results_flag  = True
    stored_propulsor_tag = propulsor.tag
    return thrust, moment, P_EHD, P_bus, stored_results_flag, stored_propulsor_tag

# ----------------------------------------------------------------------------------------------------------------------
#  reuse_stored_ehd_thruster_data
# ----------------------------------------------------------------------------------------------------------------------
def reuse_stored_ehd_thruster_data(propulsor, state, network, stored_propulsor_tag, center_of_gravity=[[0.0, 0.0, 0.0]]):
    """
    Reuses the results of an identical EHD thruster and recomputes only the moment (spec 3.2 item 6).

    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Powertrain.Propulsors.EHD_Thruster
    state : RCAIDE.Framework.Mission.Common.State
    network : RCAIDE.Framework.Networks.Network
    stored_propulsor_tag : str
        Tag of the thruster whose results are copied
    center_of_gravity : list of lists, optional

    Returns
    -------
    thrust, moment, power, power_elec : numpy.ndarray
        As in compute_ehd_thruster_performance
    """
    conditions = state.conditions
    stored     = network.propulsors[stored_propulsor_tag]
    conditions.energy.converters[propulsor.electrode_array.tag]        = deepcopy(conditions.energy.converters[stored.electrode_array.tag])
    conditions.energy.modulators[propulsor.high_voltage_converter.tag] = deepcopy(conditions.energy.modulators[stored.high_voltage_converter.tag])

    p_cond  = conditions.energy.propulsors[propulsor.tag]
    p_0     = conditions.energy.propulsors[stored_propulsor_tag]
    for key in ('thrust', 'power', 'net_thrust', 'electrode_power', 'bus_power', 'thrust_to_power_ratio'):
        p_cond[key] = deepcopy(p_0[key])
    p_cond.moment = _compute_moment(propulsor, p_cond.thrust, center_of_gravity)
    return p_cond.thrust, p_cond.moment, p_cond.power, p_cond.bus_power

def _compute_moment(propulsor, thrust, center_of_gravity):
    arm = np.array(propulsor.origin[0], dtype=float) - np.array(center_of_gravity[0], dtype=float)
    return np.cross(arm * np.ones_like(thrust), thrust)
