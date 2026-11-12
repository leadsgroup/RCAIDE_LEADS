# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbofan/solve_turbofan_offdesign_jax.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import array_namespace
from RCAIDE.Library.Methods.Powertrain.Propulsors.Common.solve_offdesign_matching_jax import solve_offdesign_matching_jax, robust_states_from_points
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_OffDesign_Matching import (
    pack_design_constants, pack_reference_point, solve_turbofan_offdesign_robust)

import numpy as np

# matching quantities carried between iterations, as named in the robust solver's result
STATE_FIELDS = ['tau_f', 'tau_tL', 'pi_tL', 'alpha', 'mass_flow_rate']
STATE_SIZE   = len(STATE_FIELDS)

# ----------------------------------------------------------------------------------------------------------------------
#  Pieces of the matching equations, as in solve_turbofan_offdesign_kernel, with its branches as where
# ----------------------------------------------------------------------------------------------------------------------
def mass_flow_parameter(mach_number, gamma, gas_constant, xp):
    """Mass flow parameter of a stream at a Mach number."""
    return xp.sqrt(gamma / gas_constant) * mach_number * (1 + (gamma - 1) / 2 * mach_number ** 2) ** (-(gamma + 1) / (2 * (gamma - 1)))

def nozzle_state(pressure_ratio, gamma, xp):
    """Exit static-to-ambient pressure ratio and exit Mach number of a nozzle: no flow, subsonic or choked."""
    critical_ratio = ((gamma + 1) / 2) ** (gamma / (gamma - 1))
    subsonic_mach  = xp.sqrt(2 / (gamma - 1) * (xp.maximum(pressure_ratio, 1.0) ** ((gamma - 1) / gamma) - 1))
    exit_ratio     = xp.where(pressure_ratio < critical_ratio, 1.0, pressure_ratio / critical_ratio)
    exit_mach      = xp.where(pressure_ratio < 1.0, 0.0, xp.where(pressure_ratio < critical_ratio, subsonic_mach, 1.0))
    return exit_ratio, exit_mach

def constant_efficiency_pressure_ratio(temperature_ratio, efficiency, gamma, xp):
    """Compressor pressure ratio from its temperature ratio at constant efficiency."""
    return xp.maximum(1 + efficiency * (temperature_ratio - 1), 1e-6) ** (gamma / (gamma - 1))

def exit_area(mass_flow_rate, static_pressure, static_temperature, gas_constant, gamma, mach_number, xp):
    """Nozzle exit area passing a mass flow at its exit state; a zero Mach number is replaced to keep it finite."""
    mach_number = xp.where(mach_number > 0, mach_number, 1.0)
    stagnation_pressure    = static_pressure * (1 + (gamma - 1) / 2 * mach_number ** 2) ** (gamma / (gamma - 1))
    stagnation_temperature = static_temperature * (1 + (gamma - 1) / 2 * mach_number ** 2)
    return mass_flow_rate * xp.sqrt(stagnation_temperature) / (stagnation_pressure * mass_flow_parameter(mach_number, gamma, gas_constant, xp))

def unpack(design, reference):
    """Packed design constants and reference point as named floats."""
    d = dict(zip(['gamma_c', 'gamma_t', 'cpc', 'cpt', 'pi_dmax', 'fuel_heating_value', 'eta_b', 'pi_b', 'pi_tH', 'tau_tH',
                  'pi_n', 'pi_fn', 'eta_cH', 'eta_f', 'eta_f_alone', 'fan_temperature_rise_fraction', 'eta_tL',
                  'shaft_work_specific_design'], [float(value) for value in design]))
    r = dict(zip(['M0', 'T0', 'P0', 'Tt4', 'tau_f', 'tau_tL', 'pi_tL', 'alpha', 'pi_cH', 'tau_cH', 'pi_f', 'M19', 'M9', 'm0'],
                 [float(value) for value in reference]))
    return d, r

def flight_terms(M0, T0, Tt4, d, xp):
    """Ram, inlet and burner terms set by the flight condition and the combustor exit temperature."""
    tau_r      = 1 + (d['gamma_c'] - 1) / 2 * M0 ** 2
    pi_r       = tau_r ** (d['gamma_c'] / (d['gamma_c'] - 1))
    supersonic = M0 > 1
    eta_r      = xp.where(supersonic, 1 - 0.075 * xp.where(supersonic, M0 - 1, 1.0) ** 1.35, 1.0)
    pi_d       = d['pi_dmax'] * eta_r
    tau_lambda = d['cpt'] * Tt4 / (d['cpc'] * T0)
    return tau_r, pi_r, pi_d, tau_lambda

def reference_terms(d, r):
    """The same terms at the reference point, as floats."""
    tau_rR      = 1 + (d['gamma_c'] - 1) / 2 * r['M0'] ** 2
    pi_rR       = tau_rR ** (d['gamma_c'] / (d['gamma_c'] - 1))
    eta_rR      = 1.0 if r['M0'] <= 1 else 1 - 0.075 * (r['M0'] - 1) ** 1.35
    pi_dR       = d['pi_dmax'] * eta_rR
    tau_lambdaR = d['cpt'] * r['Tt4'] / (d['cpc'] * r['T0'])
    return tau_rR, pi_rR, pi_dR, tau_lambdaR

def compressor_and_nozzles(state, inputs, d, r, xp):
    """Compressor, fan and nozzle quantities at a state, shared by one matching step and the outputs."""
    tau_f, tau_tL, pi_tL, alpha, mass_flow_estimate = state
    M0, T0, P0, Tt4, shaft_power_offtake            = inputs
    gamma_c, gamma_t                  = d['gamma_c'], d['gamma_t']
    tau_r, pi_r, pi_d, tau_lambda     = flight_terms(M0, T0, Tt4, d, xp)
    tau_rR, pi_rR, pi_dR, tau_lambdaR = reference_terms(d, r)

    X           = tau_lambda / (tau_r * tau_f)
    XR          = tau_lambdaR / (tau_rR * r['tau_f'])
    phi         = shaft_power_offtake * (1 + alpha) / mass_flow_estimate / (d['cpt'] * Tt4)
    phiR        = d['shaft_work_specific_design'] / (d['cpt'] * r['Tt4'])
    tau_cH      = 1 + (X / XR) * (r['tau_cH'] - 1) + X * (phiR - phi)
    pi_cH       = constant_efficiency_pressure_ratio(tau_cH, d['eta_cH'], gamma_c, xp)
    pi_f        = constant_efficiency_pressure_ratio(tau_f, d['eta_f'], gamma_c, xp)
    tau_f_alone = 1 + d['fan_temperature_rise_fraction'] * (tau_f - 1)
    pi_f_alone  = (1 + d['eta_f_alone'] * (tau_f_alone - 1)) ** (gamma_c / (gamma_c - 1))
    Pt19_P0     = pi_r * pi_d * pi_f_alone * d['pi_fn']
    P19_P0, M19 = nozzle_state(Pt19_P0, gamma_c, xp)
    Pt9_P0      = pi_r * pi_d * pi_f * pi_cH * d['pi_b'] * d['pi_tH'] * pi_tL * d['pi_n']
    P9_P0, M9   = nozzle_state(Pt9_P0, gamma_t, xp)
    return dict(tau_r=tau_r, pi_r=pi_r, pi_d=pi_d, tau_lambda=tau_lambda, tau_rR=tau_rR, pi_rR=pi_rR, pi_dR=pi_dR,
                tau_lambdaR=tau_lambdaR, tau_cH=tau_cH, pi_cH=pi_cH, pi_f=pi_f, tau_f_alone=tau_f_alone,
                pi_f_alone=pi_f_alone, Pt19_P0=Pt19_P0, P19_P0=P19_P0, M19=M19, Pt9_P0=Pt9_P0, P9_P0=P9_P0, M9=M9)

# ----------------------------------------------------------------------------------------------------------------------
#  turbofan_matching_step
# ----------------------------------------------------------------------------------------------------------------------
def turbofan_matching_step(state, inputs, design, reference, relaxation_factor=0.5):
    """
    One fixed-point iteration of the off-design matching of solve_turbofan_offdesign_kernel, for every control point
    at once, on NumPy or JAX.

    Inputs:
       state               (tau_f, tau_tL, pi_tL, bypass ratio, mass flow estimate), one array per quantity  [-, -, -, -, kg/s]
       inputs              (M0, T0, P0, Tt4, shaft power offtake), one array per quantity                     [-, K, Pa, K, W]
       design, reference   packed design constants and reference point (pack_design_constants, pack_reference_point)
       relaxation_factor   under-relaxation of the fan and low pressure turbine updates                       [-]

    Outputs:
       new_state           the state after the iteration
       collapsed           True where a nozzle has no flow at the current state (the kernel stops there)
    """
    tau_f, tau_tL, pi_tL, alpha, mass_flow_estimate = state
    M0, T0, P0, Tt4, shaft_power_offtake            = inputs
    xp      = array_namespace(tau_f, M0, Tt4)
    d, r    = unpack(design, reference)
    gamma_c, gamma_t = d['gamma_c'], d['gamma_t']
    R_c     = (gamma_c - 1) / gamma_c * d['cpc']
    R_t     = (gamma_t - 1) / gamma_t * d['cpt']
    s       = compressor_and_nozzles(state, inputs, d, r, xp)

    # bypass ratio and mass flow, then the relaxed fan and low pressure turbine updates
    alpha_new = r['alpha'] * (r['pi_cH'] / s['pi_cH']) * xp.sqrt(xp.maximum(
                (s['tau_lambda'] / (s['tau_r'] * tau_f)) / (s['tau_lambdaR'] / (s['tau_rR'] * r['tau_f'])), 1e-12)) * \
                (mass_flow_parameter(s['M19'], gamma_c, R_c, xp) / mass_flow_parameter(r['M19'], gamma_c, R_c, np))
    mass_flow_estimate_new = r['m0'] * ((1 + alpha_new) / (1 + r['alpha'])) * \
                (P0 * s['pi_r'] * s['pi_d'] * s['pi_f'] * s['pi_cH']) / (r['P0'] * s['pi_rR'] * s['pi_dR'] * r['pi_f'] * r['pi_cH']) * \
                xp.sqrt(r['Tt4'] / xp.maximum(Tt4, 1e-6))
    tau_f_computed = 1 + ((1 - tau_tL) / (1 - r['tau_tL'])) * ((s['tau_lambda'] / s['tau_r']) / (s['tau_lambdaR'] / s['tau_rR'])) * \
                     ((1 + r['alpha']) / (1 + alpha_new)) * (r['tau_f'] - 1)
    tau_f_new      = tau_f + relaxation_factor * (tau_f_computed - tau_f)
    tau_tL_new     = 1 - d['eta_tL'] * (1 - pi_tL ** ((gamma_t - 1) / gamma_t))
    collapsed      = (s['M9'] == 0.0) | (s['M19'] == 0.0)
    M9             = xp.where(collapsed, 1.0, s['M9'])
    pi_tL_computed = r['pi_tL'] * xp.sqrt(tau_tL_new / r['tau_tL']) * \
                     (mass_flow_parameter(r['M9'], gamma_t, R_t, np) / mass_flow_parameter(M9, gamma_t, R_t, xp))
    pi_tL_new      = pi_tL + relaxation_factor * (pi_tL_computed - pi_tL)
    return (tau_f_new, tau_tL_new, pi_tL_new, alpha_new, mass_flow_estimate_new), collapsed

# ----------------------------------------------------------------------------------------------------------------------
#  turbofan_matching_outputs
# ----------------------------------------------------------------------------------------------------------------------
def turbofan_matching_outputs(state, inputs, design, reference):
    """
    Engine outputs at a matched state, as solve_turbofan_offdesign_kernel computes them after its loop, for every
    control point at once, on NumPy or JAX.

    Inputs:
       state, inputs, design, reference   as in turbofan_matching_step

    Outputs:
       outputs   dict of arrays named as the fields of solve_turbofan_offdesign: thrust, fuel_mass_flow_rate, the
                 cycle ratios and the core and fan nozzle exit states
    """
    tau_f, tau_tL, pi_tL, alpha, mass_flow_estimate = state
    M0, T0, P0, Tt4, shaft_power_offtake            = inputs
    xp      = array_namespace(tau_f, M0, Tt4)
    d, r    = unpack(design, reference)
    gamma_c, gamma_t = d['gamma_c'], d['gamma_t']
    R_c     = (gamma_c - 1) / gamma_c * d['cpc']
    R_t     = (gamma_t - 1) / gamma_t * d['cpt']
    s       = compressor_and_nozzles(state, inputs, d, r, xp)
    V0      = xp.sqrt(gamma_c * R_c * T0) * M0

    mass_flow_rate    = r['m0'] * ((1 + alpha) / (1 + r['alpha'])) * \
                        (P0 * s['pi_r'] * s['pi_d'] * s['pi_f'] * s['pi_cH']) / (r['P0'] * s['pi_rR'] * s['pi_dR'] * r['pi_f'] * r['pi_cH']) * \
                        xp.sqrt(r['Tt4'] / xp.maximum(Tt4, 1e-6))
    tau_x             = s['tau_r'] * tau_f * s['tau_cH']
    fuel_to_air_ratio = (s['tau_lambda'] - tau_x) / (d['fuel_heating_value'] * d['eta_b'] / (d['cpc'] * T0) - s['tau_lambda'])
    T9  = (s['tau_lambda'] * d['tau_tH'] * tau_tL / ((s['Pt9_P0'] / s['P9_P0']) ** ((gamma_t - 1) / gamma_t))) * (d['cpc'] / d['cpt']) * T0
    T19 = (s['tau_r'] * s['tau_f_alone']) / ((s['Pt19_P0'] / s['P19_P0']) ** ((gamma_c - 1) / gamma_c)) * T0
    P9  = s['P9_P0'] * P0
    P19 = s['P19_P0'] * P0
    V9  = s['M9'] * xp.sqrt(gamma_t * R_t * T9)
    V19 = s['M19'] * xp.sqrt(gamma_c * R_c * T19)
    core_mass_flow_rate = mass_flow_rate / (1 + alpha)
    fan_mass_flow_rate  = mass_flow_rate * alpha / (1 + alpha)
    core_thrust = core_mass_flow_rate * ((1 + fuel_to_air_ratio) * V9 - V0) + \
                  (P9 - P0) * exit_area(core_mass_flow_rate * (1 + fuel_to_air_ratio), P9, T9, R_t, gamma_t, s['M9'], xp)
    fan_thrust  = fan_mass_flow_rate * (V19 - V0) + (P19 - P0) * exit_area(fan_mass_flow_rate, P19, T19, R_c, gamma_c, s['M19'], xp)

    return dict(tau_r = s['tau_r'], pi_r = s['pi_r'], pi_d = s['pi_d'], tau_cH = s['tau_cH'], pi_cH = s['pi_cH'],
                tau_f = tau_f, pi_f = s['pi_f'], tau_f_alone = s['tau_f_alone'], alpha = alpha,
                mass_flow_rate = mass_flow_rate, fuel_to_air_ratio = fuel_to_air_ratio,
                thrust = core_thrust + fan_thrust, fuel_mass_flow_rate = fuel_to_air_ratio * core_mass_flow_rate,
                core_nozzle_exit_velocity = V9, fan_nozzle_exit_velocity = V19,
                core_nozzle_exit_static_temperature = T9, fan_nozzle_exit_static_temperature = T19,
                core_nozzle_exit_static_pressure = P9, fan_nozzle_exit_static_pressure = P19,
                core_nozzle_exit_stagnation_temperature = T9 * (1 + (gamma_t - 1) / 2 * s['M9'] ** 2),
                fan_nozzle_exit_stagnation_temperature  = T19 * (1 + (gamma_c - 1) / 2 * s['M19'] ** 2),
                core_nozzle_exit_stagnation_pressure    = s['Pt9_P0'] * P0,
                fan_nozzle_exit_stagnation_pressure     = s['Pt19_P0'] * P0)

# ----------------------------------------------------------------------------------------------------------------------
#  solve_turbofan_offdesign_jax
# ----------------------------------------------------------------------------------------------------------------------
def solve_turbofan_offdesign_jax(design_constants, reference_point, mach_number, static_temperature, static_pressure,
                                 combustor_exit_temperature, shaft_power_offtake, relaxation_factor=0.5):
    """
    Off-design matching of every control point at once on JAX, differentiable with respect to the flight condition,
    the combustor exit temperature and the shaft power offtake.

    The matched state is the fixed point of turbofan_matching_step, solved by solve_offdesign_matching_jax: a JAX
    iteration from the reference point, solve_turbofan_offdesign_robust (continuation, as on the NumPy path) through
    a callback at points the iteration does not converge, and derivatives by the implicit function theorem.

    Inputs:
       design_constants, reference_point   offdesign_matching.design_constants and .reference_point
       mach_number, static_temperature,
       static_pressure                     freestream at each control point, 1-D                         [-, K, Pa]
       combustor_exit_temperature          Tt4 at each control point, 1-D                                 [K]
       shaft_power_offtake                 net shaft power taken from the HP spool at each point, 1-D     [W]

    Outputs:
       outputs   dict of turbofan_matching_outputs; NaN at points no solver matches
       matched   True where the point is matched
    """
    import jax.numpy as jnp

    design    = pack_design_constants(design_constants)
    reference = pack_reference_point(reference_point)
    inputs    = (mach_number, static_temperature, static_pressure, combustor_exit_temperature, shaft_power_offtake)
    n         = mach_number.shape[0]

    def step(flat):
        new_state, collapsed = turbofan_matching_step(tuple(flat.reshape(STATE_SIZE, n)), inputs, design, reference, relaxation_factor)
        return jnp.stack(new_state).ravel(), collapsed

    def residual(flat):
        return step(flat)[0] - flat

    def solve_point(i, values):
        M0, T0, P0, Tt4, offtake = (value[i] for value in values)
        return solve_turbofan_offdesign_robust(design_constants, reference_point, M0, T0, P0, Tt4, packed_design_constants=design,
                                               packed_reference_point=reference, shaft_power_offtake=offtake)
    robust_states = robust_states_from_points(solve_point, STATE_FIELDS, STATE_SIZE, n)

    initial = jnp.concatenate([value * jnp.ones(n) for value in reference[[4, 5, 6, 7, 13]]])
    flat, matched = solve_offdesign_matching_jax(step, residual, initial, robust_states, inputs, STATE_SIZE, n)

    # as in the robust NumPy solver: a combustor exit temperature not above the inlet stagnation temperature has
    # no matched operating point, though the iteration may still find a fixed point there
    gamma_c = float(design_constants.gamma_c)
    matched = matched & (combustor_exit_temperature > static_temperature * (1 + (gamma_c - 1) / 2 * mach_number ** 2))

    outputs = turbofan_matching_outputs(tuple(flat.reshape(STATE_SIZE, n)), inputs, design, reference)
    outputs = {name: jnp.where(matched, value, jnp.nan) for name, value in outputs.items()}
    return outputs, matched
