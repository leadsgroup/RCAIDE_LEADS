# RCAIDE/Library/Methods/Powertrain/Propulsors/Turboprop/solve_turboprop_offdesign_jax.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Data, array_namespace
from RCAIDE.Library.Methods.Powertrain.Propulsors.Common.solve_offdesign_matching_jax import solve_offdesign_matching_jax, robust_states_from_points
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.solve_turbofan_offdesign_jax import (
    constant_efficiency_pressure_ratio, flight_terms, reference_terms, mass_flow_parameter, nozzle_state)
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.Turboprop_OffDesign_Matching import (
    pack_turboprop_design_constants, pack_turboprop_reference_point, solve_turboprop_offdesign_robust,
    TURBOPROP_DESIGN_CONSTANT_FIELDS, TURBOPROP_REFERENCE_POINT_FIELDS)
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.compute_actuator_disk_propeller_thrust import compute_actuator_disk_propeller_thrust

import numpy as np

# matching quantities carried between iterations, as named in the robust solver's result
STATE_FIELDS = ['tau_c', 'pi_tL', 'mass_flow_rate']
STATE_SIZE   = len(STATE_FIELDS)

def unpack(design, reference):
    """Packed design constants and reference point as named floats."""
    d = dict(zip(TURBOPROP_DESIGN_CONSTANT_FIELDS, [float(value) for value in design]))
    r = dict(zip(TURBOPROP_REFERENCE_POINT_FIELDS, [float(value) for value in reference]))
    return d, r

def gas_generator(tau_c_state, mass_flow_estimate, inputs, d, r, xp):
    """Compressor temperature ratio from the work balance, its pressure ratio and the gas generator pressure ratio."""
    M0, T0, P0, Tt4, shaft_power_offtake = inputs
    M0     = xp.maximum(M0, 0.01)
    tau_r, pi_r, pi_d, tau_lambda     = flight_terms(M0, T0, Tt4, d, xp)
    tau_rR, pi_rR, pi_dR, tau_lambdaR = reference_terms(d, r)
    phi    = shaft_power_offtake / mass_flow_estimate / (d['cpt'] * Tt4)
    phiR   = d['shaft_work_specific_design'] / (d['cpt'] * r['Tt4'])
    tau_c  = 1 + ((tau_lambda / tau_r) / (tau_lambdaR / tau_rR)) * (r['tau_c'] - 1) + (tau_lambda / tau_r) * (phiR - phi)
    pi_c   = constant_efficiency_pressure_ratio(tau_c, d['eta_c'], d['gamma_c'], xp)
    gas_generator_pressure_ratio = pi_r * pi_d * pi_c * d['pi_b'] * d['pi_tH'] * d['pi_n']
    mass_flow_rate = r['m0'] * (P0 * pi_r * pi_d * pi_c) / (r['P0'] * pi_rR * pi_dR * r['pi_c']) * xp.sqrt(r['Tt4'] / xp.maximum(Tt4, 1e-6))
    return dict(M0=M0, tau_r=tau_r, pi_r=pi_r, pi_d=pi_d, tau_lambda=tau_lambda, tau_c=tau_c, pi_c=pi_c,
                gas_generator_pressure_ratio=gas_generator_pressure_ratio, mass_flow_rate=mass_flow_rate)

def free_turbine_residual(pi_tL, gas_generator_pressure_ratio, d, r, xp):
    """Free turbine and core nozzle mass flow match, Mattingly (2006) Eq. 8.52w; -inf where the nozzle has no flow."""
    gamma_t = d['gamma_t']
    R_t     = (gamma_t - 1) / gamma_t * d['cpt']
    tau_tL  = 1 - d['eta_tL'] * (1 - pi_tL ** ((gamma_t - 1) / gamma_t))
    _, M9   = nozzle_state(gas_generator_pressure_ratio * pi_tL, gamma_t, xp)
    flowing = M9 > 0.0
    match   = pi_tL - r['pi_tL'] * xp.sqrt(tau_tL / r['tau_tL']) * \
              (mass_flow_parameter(r['M9'], gamma_t, R_t, np) / mass_flow_parameter(xp.where(flowing, M9, 1.0), gamma_t, R_t, xp))
    return xp.where(flowing, match, -np.inf)

def free_turbine_pressure_ratio(gas_generator_pressure_ratio, d, r, xp):
    """
    solve_free_turbine_pressure_ratio for every point at once: bisection between no nozzle pressure to expand and no
    expansion, each point stopping as the kernel does; NaN where no match exists.
    """
    ratio  = gas_generator_pressure_ratio
    exists = (ratio > 1.0) & (free_turbine_residual(1.0 + 0 * ratio, ratio, d, r, xp) >= 0.0)
    ratio  = xp.where(exists, ratio, 2.0)
    def bisect(iteration, bracket):
        lower, upper, active = bracket
        middle = 0.5 * (lower + upper)
        active = active & (middle > lower) & (middle < upper)
        below  = free_turbine_residual(middle, ratio, d, r, xp) < 0.0
        return xp.where(active & below, middle, lower), xp.where(active & ~below, middle, upper), active

    bracket = (1.0 / ratio, 1.0 + 0 * ratio, exists)
    if xp is np:
        for iteration in range(200):
            bracket = bisect(iteration, bracket)
    else:
        import jax
        bracket = jax.lax.fori_loop(0, 200, bisect, bracket)
    return xp.where(exists, bracket[1], np.nan)

# ----------------------------------------------------------------------------------------------------------------------
#  turboprop_matching_step
# ----------------------------------------------------------------------------------------------------------------------
def turboprop_matching_step(state, inputs, design, reference):
    """
    One iteration of the off-design matching of solve_turboprop_offdesign_kernel, for every control point at once,
    on NumPy or JAX. The free turbine pressure ratio is found by bisection inside the step, as in the kernel.

    Inputs:
       state               (tau_c, pi_tL, mass flow estimate), one array per quantity   [-, -, kg/s]
       inputs              (M0, T0, P0, Tt4, shaft power offtake)                       [-, K, Pa, K, W]
       design, reference   packed design constants and reference point

    Outputs:
       new_state           the state after the iteration
       collapsed           True where no free turbine pressure ratio passes the core flow (the kernel stops there)
    """
    tau_c, pi_tL, mass_flow_estimate = state
    xp    = array_namespace(tau_c, inputs[0], inputs[3])
    d, r  = unpack(design, reference)
    g     = gas_generator(tau_c, mass_flow_estimate, inputs, d, r, xp)
    pi_tL_new = free_turbine_pressure_ratio(g['gas_generator_pressure_ratio'], d, r, xp)
    collapsed = xp.isnan(pi_tL_new)
    return (g['tau_c'], xp.where(collapsed, pi_tL, pi_tL_new), g['mass_flow_rate']), collapsed

def turboprop_matching_residual(state, inputs, design, reference):
    """The matching equations at a state, zero at the matched point: compressor work, free turbine match, mass flow."""
    tau_c, pi_tL, mass_flow_estimate = state
    xp   = array_namespace(tau_c, inputs[0], inputs[3])
    d, r = unpack(design, reference)
    g    = gas_generator(tau_c, mass_flow_estimate, inputs, d, r, xp)
    # the gas generator pressure ratio at the state's own compressor temperature ratio
    pi_c  = constant_efficiency_pressure_ratio(tau_c, d['eta_c'], d['gamma_c'], xp)
    ratio = g['pi_r'] * g['pi_d'] * pi_c * d['pi_b'] * d['pi_tH'] * d['pi_n']
    match = free_turbine_residual(pi_tL, ratio, d, r, xp)
    tau_rR, pi_rR, pi_dR, tau_lambdaR = reference_terms(d, r)
    mass_flow_rate = r['m0'] * (inputs[2] * g['pi_r'] * g['pi_d'] * pi_c) / (r['P0'] * pi_rR * pi_dR * r['pi_c']) * \
                     xp.sqrt(r['Tt4'] / xp.maximum(inputs[3], 1e-6))
    return (g['tau_c'] - tau_c, xp.where(xp.isfinite(match), match, -1.0), mass_flow_rate - mass_flow_estimate)

# ----------------------------------------------------------------------------------------------------------------------
#  turboprop_matching_outputs
# ----------------------------------------------------------------------------------------------------------------------
def turboprop_matching_outputs(state, inputs, design, reference):
    """
    Engine and propeller outputs at a state (tau_c, pi_tL, mass flow), as solve_turboprop_offdesign_kernel computes
    them after its loop, for every control point at once, on NumPy or JAX.

    Outputs:
       outputs   Data of arrays named as the fields of solve_turboprop_offdesign
    """
    tau_c, pi_tL, mass_flow_rate     = state
    M0, T0, P0, Tt4, shaft_power_offtake = inputs
    xp      = array_namespace(tau_c, M0, Tt4)
    d, r    = unpack(design, reference)
    gamma_c, gamma_t = d['gamma_c'], d['gamma_t']
    R_c     = (gamma_c - 1) / gamma_c * d['cpc']
    R_t     = (gamma_t - 1) / gamma_t * d['cpt']
    M0      = xp.maximum(M0, 0.01)
    tau_r, pi_r, pi_d, tau_lambda = flight_terms(M0, T0, Tt4, d, xp)
    a0      = xp.sqrt(gamma_c * R_c * T0)
    V0      = a0 * M0
    pi_c    = constant_efficiency_pressure_ratio(tau_c, d['eta_c'], gamma_c, xp)
    tau_tL  = 1 - d['eta_tL'] * (1 - pi_tL ** ((gamma_t - 1) / gamma_t))
    Pt9_P0  = pi_r * pi_d * pi_c * d['pi_b'] * d['pi_tH'] * d['pi_n'] * pi_tL
    P9_P0, M9 = nozzle_state(Pt9_P0, gamma_t, xp)
    tau_x   = tau_r * tau_c
    f       = (tau_lambda - tau_x) / (d['fuel_heating_value'] * d['eta_b'] / (d['cpc'] * T0) - tau_lambda)
    Pt9_P9  = Pt9_P0 / P9_P0
    T9      = (tau_lambda * d['tau_tH'] * tau_tL / (Pt9_P9 ** ((gamma_t - 1) / gamma_t))) * (d['cpc'] / d['cpt']) * T0
    P9      = P9_P0 * P0
    a9      = xp.sqrt(gamma_t * R_t * T9)
    V9      = M9 * a9
    Ccore   = (gamma_c - 1) * M0 * ((1 + f) * (V9 / a0) - M0 + (1 + f) * (R_t / R_c) * ((T9 / T0) / (V9 / a0)) * ((1 - P0 / P9) / gamma_c))
    shaft_power      = (1 + f) * d['cpt'] * Tt4 * d['tau_tH'] * (1 - tau_tL) * d['eta_mL'] * d['eta_gearbox'] * mass_flow_rate
    propeller_thrust = compute_actuator_disk_propeller_thrust(shaft_power, V0, P0 / (R_c * T0), d['propeller_disk_area'],
                                                              d['propeller_polytropic_efficiency'])
    thrust              = Ccore * d['cpc'] * T0 / V0 * mass_flow_rate + propeller_thrust
    fuel_mass_flow_rate = f * mass_flow_rate
    return Data(tau_r = tau_r, pi_r = pi_r, pi_d = pi_d, tau_c = tau_c, pi_c = pi_c, tau_tL = tau_tL, pi_tL = pi_tL, M9 = M9,
                mass_flow_rate = mass_flow_rate, fuel_to_air_ratio = f, thrust = thrust, fuel_mass_flow_rate = fuel_mass_flow_rate,
                shaft_power = shaft_power, power = propeller_thrust * V0,
                core_nozzle_exit_velocity = V9, core_nozzle_exit_static_temperature = T9, core_nozzle_exit_static_pressure = P9,
                core_nozzle_exit_stagnation_temperature = T9 * (1 + (gamma_t - 1) / 2 * M9 ** 2),
                core_nozzle_exit_stagnation_pressure = Pt9_P0 * P0)

# ----------------------------------------------------------------------------------------------------------------------
#  solve_turboprop_offdesign_jax
# ----------------------------------------------------------------------------------------------------------------------
def solve_turboprop_offdesign_jax(design_constants, reference_point, mach_number, static_temperature, static_pressure,
                                  combustor_exit_temperature, shaft_power_offtake):
    """
    Off-design matching of every control point at once on JAX, differentiable with respect to the flight condition,
    the combustor exit temperature and the shaft power offtake (solve_offdesign_matching_jax, with
    solve_turboprop_offdesign_robust for points the JAX iteration does not converge). The forward iteration finds the
    free turbine pressure ratio by bisection; derivatives come from the matching equations themselves.

    Inputs:
       design_constants, reference_point   offdesign_matching.design_constants and .reference_point
       mach_number, static_temperature,
       static_pressure                     freestream at each control point, 1-D                         [-, K, Pa]
       combustor_exit_temperature          Tt4 at each control point, 1-D                                 [K]
       shaft_power_offtake                 net shaft power taken from the gas generator at each point     [W]

    Outputs:
       outputs   Data of turboprop_matching_outputs; NaN at points no solver matches
       matched   True where the point is matched
    """
    import jax.numpy as jnp

    design    = pack_turboprop_design_constants(design_constants)
    reference = pack_turboprop_reference_point(reference_point)
    inputs    = (mach_number, static_temperature, static_pressure, combustor_exit_temperature, shaft_power_offtake)
    n         = mach_number.shape[0]

    def step(flat):
        new_state, collapsed = turboprop_matching_step(tuple(flat.reshape(STATE_SIZE, n)), inputs, design, reference)
        return jnp.stack(new_state).ravel(), collapsed

    def residual(flat):
        return jnp.stack(turboprop_matching_residual(tuple(flat.reshape(STATE_SIZE, n)), inputs, design, reference)).ravel()

    def solve_point(i, values):
        M0, T0, P0, Tt4, offtake = (value[i] for value in values)
        return solve_turboprop_offdesign_robust(design_constants, reference_point, M0, T0, P0, Tt4, packed_design_constants=design,
                                                packed_reference_point=reference, shaft_power_offtake=offtake)
    robust_states = robust_states_from_points(solve_point, STATE_FIELDS, STATE_SIZE, n)

    initial       = jnp.concatenate([value * jnp.ones(n) for value in reference[np.array([4, 6, 9])]])
    flat, matched = solve_offdesign_matching_jax(step, residual, initial, robust_states, inputs, STATE_SIZE, n)

    # as in the robust NumPy solver: a combustor exit temperature not above the inlet stagnation temperature has
    # no matched operating point
    gamma_c = float(design_constants.gamma_c)
    matched = matched & (combustor_exit_temperature > static_temperature * (1 + (gamma_c - 1) / 2 * mach_number ** 2))

    outputs = turboprop_matching_outputs(tuple(flat.reshape(STATE_SIZE, n)), inputs, design, reference)
    for name in list(outputs.keys()):
        outputs[name] = jnp.where(matched, outputs[name], jnp.nan)
    return outputs, matched
