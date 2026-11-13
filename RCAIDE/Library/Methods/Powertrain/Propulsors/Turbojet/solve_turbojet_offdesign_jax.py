# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbojet/solve_turbojet_offdesign_jax.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Data, array_namespace
from RCAIDE.Library.Methods.Powertrain.Propulsors.Common.solve_offdesign_matching_jax import solve_offdesign_matching_jax, robust_states_from_points
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.solve_turbofan_offdesign_jax import (
    constant_efficiency_pressure_ratio, flight_terms, reference_terms)
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.Turbojet_OffDesign_Matching import (
    pack_turbojet_design_constants, pack_turbojet_reference_point, solve_turbojet_offdesign_robust,
    TURBOJET_DESIGN_CONSTANT_FIELDS, TURBOJET_REFERENCE_POINT_FIELDS)

import numpy as np

# matching quantities carried between iterations, as named in the robust solver's result
STATE_FIELDS = ['tau_c', 'tau_tL', 'pi_tL', 'mass_flow_rate']
STATE_SIZE   = len(STATE_FIELDS)

def unpack(design, reference):
    """Packed design constants and reference point as named floats."""
    d = dict(zip(TURBOJET_DESIGN_CONSTANT_FIELDS, [float(value) for value in design]))
    r = dict(zip(TURBOJET_REFERENCE_POINT_FIELDS, [float(value) for value in reference]))
    return d, r

def compressors(state, inputs, d, r, xp):
    """Compressor quantities and nozzle pressure ratio at a state, shared by one matching step and the outputs."""
    tau_c, tau_tL, pi_tL, mass_flow_estimate = state
    M0, T0, P0, Tt4, shaft_power_offtake     = inputs
    tau_r, pi_r, pi_d, tau_lambda     = flight_terms(M0, T0, Tt4, d, xp)
    tau_rR, pi_rR, pi_dR, tau_lambdaR = reference_terms(d, r)
    X      = tau_lambda / (tau_r * tau_c)
    XR     = tau_lambdaR / (tau_rR * r['tau_c'])
    phi    = shaft_power_offtake / mass_flow_estimate / (d['cpt'] * Tt4)
    phiR   = d['shaft_work_specific_design'] / (d['cpt'] * r['Tt4'])
    tau_cH = 1 + (X / XR) * (r['tau_cH'] - 1) + X * (phiR - phi)
    pi_cH  = constant_efficiency_pressure_ratio(tau_cH, d['eta_cH'], d['gamma_c'], xp)
    pi_c   = constant_efficiency_pressure_ratio(tau_c, d['eta_c'], d['gamma_c'], xp)
    Pt9_P0 = pi_r * pi_d * pi_c * pi_cH * d['pi_b'] * d['pi_tH'] * pi_tL * d['pi_n']
    mass_flow_rate = r['m0'] * (P0 * pi_r * pi_d * pi_c * pi_cH) / (r['P0'] * pi_rR * pi_dR * r['pi_c'] * r['pi_cH']) * \
                     xp.sqrt(r['Tt4'] / xp.maximum(Tt4, 1e-6))
    return dict(tau_r=tau_r, pi_r=pi_r, pi_d=pi_d, tau_lambda=tau_lambda, tau_rR=tau_rR, tau_lambdaR=tau_lambdaR,
                tau_cH=tau_cH, pi_cH=pi_cH, pi_c=pi_c, Pt9_P0=Pt9_P0, mass_flow_rate=mass_flow_rate)

# ----------------------------------------------------------------------------------------------------------------------
#  turbojet_matching_step
# ----------------------------------------------------------------------------------------------------------------------
def turbojet_matching_step(state, inputs, design, reference, relaxation_factor=0.5):
    """
    One fixed-point iteration of the off-design matching of solve_turbojet_offdesign_kernel, for every control point
    at once, on NumPy or JAX.

    Inputs:
       state               (tau_c, tau_tL, pi_tL, mass flow estimate), one array per quantity   [-, -, -, kg/s]
       inputs              (M0, T0, P0, Tt4, shaft power offtake), one array per quantity       [-, K, Pa, K, W]
       design, reference   packed design constants and reference point
       relaxation_factor   under-relaxation of the compressor and low pressure turbine updates  [-]

    Outputs:
       new_state           the state after the iteration
       collapsed           True where the nozzle has no flow at the current state (the kernel stops there)
    """
    tau_c, tau_tL, pi_tL, mass_flow_estimate = state
    xp   = array_namespace(tau_c, inputs[0], inputs[3])
    d, r = unpack(design, reference)
    s    = compressors(state, inputs, d, r, xp)
    collapsed      = s['Pt9_P0'] < 1.0
    pi_tL_computed = r['pi_tL'] * xp.sqrt(tau_tL / r['tau_tL'])
    pi_tL_new      = pi_tL + relaxation_factor * (pi_tL_computed - pi_tL)
    tau_tL_new     = 1 - d['eta_tL'] * (1 - pi_tL_new ** ((d['gamma_t'] - 1) / d['gamma_t']))
    tau_c_computed = 1 + ((1 - tau_tL_new) / (1 - r['tau_tL'])) * ((s['tau_lambda'] / s['tau_r']) / (s['tau_lambdaR'] / s['tau_rR'])) * \
                     (r['tau_c'] - 1)
    tau_c_new      = tau_c + relaxation_factor * (tau_c_computed - tau_c)
    return (tau_c_new, tau_tL_new, pi_tL_new, s['mass_flow_rate']), collapsed

# ----------------------------------------------------------------------------------------------------------------------
#  turbojet_matching_outputs
# ----------------------------------------------------------------------------------------------------------------------
def turbojet_matching_outputs(state, inputs, design, reference):
    """
    Dry engine outputs at a matched state, as solve_turbojet_offdesign_kernel computes them after its loop, for every
    control point at once, on NumPy or JAX.

    Outputs:
       outputs   Data of arrays named as the fields of solve_turbojet_offdesign
    """
    tau_c, tau_tL, pi_tL, mass_flow_estimate = state
    M0, T0, P0, Tt4, shaft_power_offtake     = inputs
    xp      = array_namespace(tau_c, M0, Tt4)
    d, r    = unpack(design, reference)
    gamma_c = d['gamma_c']
    R_c     = (gamma_c - 1) / gamma_c * d['cpc']
    s       = compressors(state, inputs, d, r, xp)
    V0      = xp.sqrt(gamma_c * R_c * T0) * M0
    M9      = xp.sqrt(2 / (gamma_c - 1) * (xp.maximum(s['Pt9_P0'], 1.0) ** ((gamma_c - 1) / gamma_c) - 1))
    tau_x   = s['tau_r'] * tau_c * s['tau_cH']
    fuel_to_air_ratio = (s['tau_lambda'] - tau_x) / (d['fuel_heating_value'] * d['eta_b'] / (d['cpc'] * T0) - s['tau_lambda'])
    Tt9     = Tt4 * d['tau_tH'] * tau_tL
    T9      = Tt9 / (1 + (gamma_c - 1) / 2 * M9 ** 2)
    V9      = M9 * xp.sqrt(gamma_c * R_c * T9)
    mass_flow_rate      = s['mass_flow_rate']
    thrust              = mass_flow_rate * ((1 + fuel_to_air_ratio) * V9 - V0)
    fuel_mass_flow_rate = fuel_to_air_ratio * mass_flow_rate
    return Data(tau_r = s['tau_r'], pi_r = s['pi_r'], pi_d = s['pi_d'], tau_lambda = s['tau_lambda'], tau_c = tau_c,
                pi_c = s['pi_c'], tau_tL = tau_tL, M9 = M9, stagnation_to_ambient_core_nozzle_pressure_ratio = s['Pt9_P0'],
                mass_flow_rate = mass_flow_rate, fuel_to_air_ratio = fuel_to_air_ratio, thrust = thrust,
                fuel_mass_flow_rate = fuel_mass_flow_rate,
                specific_fuel_consumption = xp.where(thrust > 0, fuel_mass_flow_rate / xp.where(thrust > 0, thrust, 1.0), np.nan),
                core_nozzle_exit_velocity = V9, core_nozzle_exit_static_temperature = T9,
                core_nozzle_exit_static_pressure = P0 + 0 * T9, core_nozzle_exit_stagnation_temperature = Tt9,
                core_nozzle_exit_stagnation_pressure = s['Pt9_P0'] * P0)

# ----------------------------------------------------------------------------------------------------------------------
#  solve_turbojet_offdesign_jax
# ----------------------------------------------------------------------------------------------------------------------
def solve_turbojet_offdesign_jax(design_constants, reference_point, mach_number, static_temperature, static_pressure,
                                 combustor_exit_temperature, shaft_power_offtake, relaxation_factor=0.5):
    """
    Dry off-design matching of every control point at once on JAX, differentiable with respect to the flight
    condition, the combustor exit temperature and the shaft power offtake (solve_offdesign_matching_jax, with
    solve_turbojet_offdesign_robust for points the JAX iteration does not converge).

    Inputs:
       design_constants, reference_point   offdesign_matching.design_constants and .reference_point
       mach_number, static_temperature,
       static_pressure                     freestream at each control point, 1-D                         [-, K, Pa]
       combustor_exit_temperature          Tt4 at each control point, 1-D                                 [K]
       shaft_power_offtake                 net shaft power taken from the gas generator at each point     [W]

    Outputs:
       outputs   Data of turbojet_matching_outputs; NaN at points no solver matches
       matched   True where the point is matched
    """
    import jax.numpy as jnp

    design    = pack_turbojet_design_constants(design_constants)
    reference = pack_turbojet_reference_point(reference_point)
    inputs    = (mach_number, static_temperature, static_pressure, combustor_exit_temperature, shaft_power_offtake)
    n         = mach_number.shape[0]

    def step(flat):
        new_state, collapsed = turbojet_matching_step(tuple(flat.reshape(STATE_SIZE, n)), inputs, design, reference, relaxation_factor)
        return jnp.stack(new_state).ravel(), collapsed

    def residual(flat):
        return step(flat)[0] - flat

    def solve_point(i, values):
        M0, T0, P0, Tt4, offtake = (value[i] for value in values)
        return solve_turbojet_offdesign_robust(design_constants, reference_point, M0, T0, P0, Tt4, packed_design_constants=design,
                                               packed_reference_point=reference, shaft_power_offtake=offtake)
    robust_states = robust_states_from_points(solve_point, STATE_FIELDS, STATE_SIZE, n)

    initial       = jnp.concatenate([value * jnp.ones(n) for value in reference[[4, 5, 6, 10]]])
    flat, matched = solve_offdesign_matching_jax(step, residual, initial, robust_states, inputs, STATE_SIZE, n)

    # as in the robust NumPy solver: a combustor exit temperature not above the inlet stagnation temperature has
    # no matched operating point, though the iteration may still find a fixed point there
    gamma_c = float(design_constants.gamma_c)
    matched = matched & (combustor_exit_temperature > static_temperature * (1 + (gamma_c - 1) / 2 * mach_number ** 2))

    outputs = turbojet_matching_outputs(tuple(flat.reshape(STATE_SIZE, n)), inputs, design, reference)
    for name in list(outputs.keys()):
        outputs[name] = jnp.where(matched, outputs[name], jnp.nan)
    return outputs, matched
