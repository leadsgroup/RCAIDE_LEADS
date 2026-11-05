# RCAIDE/Library/Methods/Powertrain/Propulsors/Turboprop/Turboprop_OffDesign_Matching.py
#
#
# Created:  Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_OffDesign_Matching import (
    mfp, compressor_pressure_ratio, nozzle_state, OffDesignMatchingError, njit, constant_efficiency_pressure_ratio_kernel,
    nozzle_state_kernel, mass_flow_parameter_kernel)
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.compute_actuator_disk_propeller_thrust import actuator_disk_propeller_thrust

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  design_constants / reference_point schema -- see design_turboprop_offdesign_matching
# ----------------------------------------------------------------------------------------------------------------------
# design_constants: gamma_c/gamma_t, cpc/cpt, fuel_heating_value, pi_dmax, pi_b, pi_n,
# tau_tH/pi_tH (gas-generator turbine, held constant), eta_c, eta_b, eta_mH, eta_mL, eta_tL,
# eta_prop, eta_gearbox, shaft_work_specific_design (gas-generator accessory offtake,
# 0 if none), propeller_disk_area, propeller_polytropic_efficiency (the propeller as an
# actuator disk, see compute_actuator_disk_propeller_thrust) -- all SI units.
# reference_point: M0, T0, P0, Tt4, pi_c/tau_c, tau_tL/pi_tL, M9, m0, F (design-point state;
# every off-design solve scales relative to this).

# ----------------------------------------------------------------------------------------------------------------------
#  compiled matching kernel
# ----------------------------------------------------------------------------------------------------------------------
# The same equations as solve_turboprop_offdesign, on plain floats so they compile with numba (see the turbofan
# kernel in Turbofan_OffDesign_Matching). design_constants and reference_point are packed in the orders below.
TURBOPROP_DESIGN_CONSTANT_FIELDS = ['gamma_c', 'gamma_t', 'cpc', 'cpt', 'pi_dmax', 'fuel_heating_value', 'eta_b', 'pi_b',
                                    'pi_tH', 'tau_tH', 'pi_n', 'eta_c', 'eta_tL', 'eta_gearbox', 'eta_mL',
                                    'shaft_work_specific_design', 'propeller_disk_area', 'propeller_polytropic_efficiency']
TURBOPROP_REFERENCE_POINT_FIELDS = ['M0', 'T0', 'P0', 'Tt4', 'tau_c', 'tau_tL', 'pi_tL', 'pi_c', 'M9', 'm0']
TURBOPROP_KERNEL_OUTPUT_FIELDS   = ['tau_r', 'pi_r', 'pi_d', 'tau_lambda', 'tau_c', 'pi_c', 'tau_tL', 'pi_tL', 'M9',
                                    'stagnation_to_ambient_core_nozzle_pressure_ratio', 'mass_flow_rate',
                                    'fuel_to_air_ratio', 'thrust', 'fuel_mass_flow_rate', 'specific_fuel_consumption',
                                    'power', 'iterations', 'converged', 'convergence_delta', 'core_nozzle_exit_velocity',
                                    'core_nozzle_exit_static_temperature', 'core_nozzle_exit_static_pressure',
                                    'core_nozzle_exit_stagnation_temperature', 'core_nozzle_exit_stagnation_pressure',
                                    'eta_c_used', 'shaft_power', 'collapsed']

def pack_turboprop_design_constants(design_constants):
    """Design constants as a float array in TURBOPROP_DESIGN_CONSTANT_FIELDS order (for the compiled kernel)."""
    return np.array([float(getattr(design_constants, field, 0.0)) for field in TURBOPROP_DESIGN_CONSTANT_FIELDS])

def pack_turboprop_reference_point(reference_point):
    """Reference point as a float array in TURBOPROP_REFERENCE_POINT_FIELDS order (for the compiled kernel)."""
    return np.array([float(reference_point[field]) for field in TURBOPROP_REFERENCE_POINT_FIELDS])

@njit(cache=True)
def free_turbine_mass_flow_residual(pi_tL, gas_generator_pressure_ratio, gamma_t, Rt, eta_tL, pi_tLR, tau_tLR, mfp_ref_M9):
    """
    Residual of the free-turbine/core-nozzle mass-flow match (Mattingly, Elements of Propulsion, 2006, Eq. 8.52w) at a trial
    free-turbine pressure ratio; gas_generator_pressure_ratio is Pt9/P0 without the free turbine.
    """
    tau_tL = 1 - eta_tL * (1 - pi_tL ** ((gamma_t - 1) / gamma_t))
    P9_P0, M9 = nozzle_state_kernel(gas_generator_pressure_ratio * pi_tL, gamma_t)
    if M9 == 0.0:
        return -np.inf
    return pi_tL - pi_tLR * np.sqrt(tau_tL / tau_tLR) * (mfp_ref_M9 / mass_flow_parameter_kernel(M9, gamma_t, Rt))

@njit(cache=True)
def solve_free_turbine_pressure_ratio(gas_generator_pressure_ratio, gamma_t, Rt, eta_tL, pi_tLR, tau_tLR, mfp_ref_M9):
    """
    Free-turbine pressure ratio matching the core nozzle mass flow (Mattingly, Elements of Propulsion, 2006,
    Eq. 8.52w), by bisection
    between the ratio that leaves the nozzle no pressure to expand (Pt9/P0 = 1, where the residual tends
    to minus infinity) and no expansion (pi_tL = 1). Returns NaN when no match exists, i.e. the gas
    generator cannot both drive the free turbine and push its flow through the nozzle.
    """
    if gas_generator_pressure_ratio <= 1.0:
        return np.nan
    upper = 1.0
    if free_turbine_mass_flow_residual(upper, gas_generator_pressure_ratio, gamma_t, Rt, eta_tL, pi_tLR, tau_tLR, mfp_ref_M9) < 0.0:
        return np.nan
    lower = 1.0 / gas_generator_pressure_ratio
    for iteration in range(200):
        middle = 0.5 * (lower + upper)
        if middle <= lower or middle >= upper:
            break
        if free_turbine_mass_flow_residual(middle, gas_generator_pressure_ratio, gamma_t, Rt, eta_tL, pi_tLR, tau_tLR, mfp_ref_M9) < 0.0:
            lower = middle
        else:
            upper = middle
    return upper

@njit(cache=True)
def solve_turboprop_offdesign_kernel(design, reference, mach_number, T0, P0, Tt4, tolerance, max_iterations,
                                     guess_tau_c, guess_tau_tL, guess_pi_tL, shaft_power_offtake):
    """
    Compiled solve_turboprop_offdesign: design and reference are the packed design constants and reference
    point, and the result is a float array in TURBOPROP_KERNEL_OUTPUT_FIELDS order. A NaN guess starts from
    the reference point, and a NaN shaft_power_offtake uses the design-point offtake.
    """
    gamma_c, gamma_t, cpc, cpt, pi_dmax, fuel_heating_value, eta_b, pi_b = design[0], design[1], design[2], design[3], design[4], design[5], design[6], design[7]
    pi_tH, tau_tH, pi_n, eta_c, eta_tL, eta_gearbox, eta_mL = design[8], design[9], design[10], design[11], design[12], design[13], design[14]
    shaft_work_specific_design, propeller_disk_area, propeller_polytropic_efficiency = design[15], design[16], design[17]
    M0R, T0R, P0R, Tt4R, tau_cR, tau_tLR, pi_tLR, pi_cR, M9R, m0R = reference[0], reference[1], reference[2], reference[3], reference[4], reference[5], reference[6], reference[7], reference[8], reference[9]

    M0 = max(mach_number, 0.01)
    Rc = (gamma_c - 1) / gamma_c * cpc
    Rt = (gamma_t - 1) / gamma_t * cpt
    a0 = np.sqrt(gamma_c * Rc * T0)
    V0 = a0 * M0
    tau_r = 1 + (gamma_c - 1) / 2 * M0 ** 2
    pi_r = tau_r ** (gamma_c / (gamma_c - 1))
    eta_r = 1.0 if M0 <= 1 else 1 - 0.075 * (M0 - 1) ** 1.35
    pi_d = pi_dmax * eta_r
    tau_lambda = cpt * Tt4 / (cpc * T0)
    tau_rR = 1 + (gamma_c - 1) / 2 * M0R ** 2
    pi_rR = tau_rR ** (gamma_c / (gamma_c - 1))
    eta_rR = 1.0 if M0R <= 1 else 1 - 0.075 * (M0R - 1) ** 1.35
    pi_dR = pi_dmax * eta_rR
    tau_lambdaR = cpt * Tt4R / (cpc * T0R)

    if np.isnan(guess_tau_c):
        tau_c, tau_tL, pi_tL = tau_cR, tau_tLR, pi_tLR
    else:
        tau_c, tau_tL, pi_tL = guess_tau_c, guess_tau_tL, guess_pi_tL

    P_offtake = shaft_work_specific_design * m0R if np.isnan(shaft_power_offtake) else shaft_power_offtake
    phiR = shaft_work_specific_design / (cpt * Tt4R)
    mass_flow_rate_estimate = m0R
    converged = False
    collapsed = False
    mfp_ref_M9 = mass_flow_parameter_kernel(M9R, gamma_t, Rt)
    pi_c = pi_cR
    Pt9_P0 = 1.0
    M9 = M9R
    tau_tL_prev = tau_tL
    iteration = 0
    for i in range(max_iterations):
        iteration = i
        tau_tL_prev = tau_tL
        tau_c_prev = tau_c
        tau_cH_ratio = (tau_lambda / tau_r) / (tau_lambdaR / tau_rR)
        shaft_work_specific = P_offtake / mass_flow_rate_estimate
        phi = shaft_work_specific / (cpt * Tt4)
        tau_c_new = 1 + tau_cH_ratio * (tau_cR - 1) + (tau_lambda / tau_r) * (phiR - phi)
        pi_c = constant_efficiency_pressure_ratio_kernel(tau_c_new, eta_c, gamma_c)
        tau_c = tau_c_new
        gas_generator_pressure_ratio = pi_r * pi_d * pi_c * pi_b * pi_tH * pi_n
        pi_tL_matched = solve_free_turbine_pressure_ratio(gas_generator_pressure_ratio, gamma_t, Rt, eta_tL, pi_tLR,
                                                          tau_tLR, mfp_ref_M9)
        if np.isnan(pi_tL_matched):
            Pt9_P0 = gas_generator_pressure_ratio * pi_tL
            collapsed = True
            break
        pi_tL = pi_tL_matched
        tau_tL = 1 - eta_tL * (1 - pi_tL ** ((gamma_t - 1) / gamma_t))
        mass_flow_rate_estimate = m0R * (P0 * pi_r * pi_d * pi_c) / (P0R * pi_rR * pi_dR * pi_cR) * \
            np.sqrt(Tt4R / max(Tt4, 1e-6))
        if i > 0 and abs(tau_tL - tau_tL_prev) < tolerance and abs(tau_c - tau_c_prev) < tolerance:
            converged = True
            break

    out = np.full(27, np.nan)
    out[0], out[1], out[2], out[3] = tau_r, pi_r, pi_d, tau_lambda
    out[4], out[5], out[6], out[7] = tau_c, pi_c, tau_tL, pi_tL
    out[24] = eta_c
    if collapsed:
        out[9] = Pt9_P0
        out[16], out[17], out[18] = iteration + 1, 0.0, np.inf
        out[26] = 1.0
        return out

    tau_cH_ratio = (tau_lambda / tau_r) / (tau_lambdaR / tau_rR)
    shaft_work_specific = P_offtake / mass_flow_rate_estimate
    phi = shaft_work_specific / (cpt * Tt4)
    tau_c_new = 1 + tau_cH_ratio * (tau_cR - 1) + (tau_lambda / tau_r) * (phiR - phi)
    pi_c = constant_efficiency_pressure_ratio_kernel(tau_c_new, eta_c, gamma_c)
    tau_c = tau_c_new
    gas_generator_pressure_ratio = pi_r * pi_d * pi_c * pi_b * pi_tH * pi_n
    pi_tL_matched = solve_free_turbine_pressure_ratio(gas_generator_pressure_ratio, gamma_t, Rt, eta_tL, pi_tLR, tau_tLR,
                                                      mfp_ref_M9)
    if not np.isnan(pi_tL_matched):
        pi_tL = pi_tL_matched
        tau_tL = 1 - eta_tL * (1 - pi_tL ** ((gamma_t - 1) / gamma_t))
    Pt9_P0 = gas_generator_pressure_ratio * pi_tL
    P9_P0, M9 = nozzle_state_kernel(Pt9_P0, gamma_t)
    mass_flow_rate = m0R * (P0 * pi_r * pi_d * pi_c) / (P0R * pi_rR * pi_dR * pi_cR) * np.sqrt(Tt4R / max(Tt4, 1e-6))
    tau_x = tau_r * tau_c
    fuel_to_air_ratio = (tau_lambda - tau_x) / (fuel_heating_value * eta_b / (cpc * T0) - tau_lambda)
    Pt9_P9 = Pt9_P0 / P9_P0
    T9_T0 = (tau_lambda * tau_tH * tau_tL / (Pt9_P9 ** ((gamma_t - 1) / gamma_t))) * (cpc / cpt)
    T9 = T9_T0 * T0
    P9 = P9_P0 * P0
    a9 = np.sqrt(gamma_t * Rt * T9)
    V9 = M9 * a9
    f = fuel_to_air_ratio
    Ccore = (gamma_c - 1) * M0 * (
        (1 + f) * (V9 / a0) - M0 +
        (1 + f) * (Rt / Rc) * ((T9 / T0) / (V9 / a0)) * ((1 - P0 / P9) / gamma_c)
    )
    shaft_power = (1 + f) * cpt * Tt4 * tau_tH * (1 - tau_tL) * eta_mL * eta_gearbox * mass_flow_rate
    propeller_thrust = actuator_disk_propeller_thrust(shaft_power, V0, P0 / (Rc * T0), propeller_disk_area,
                                                      propeller_polytropic_efficiency)
    thrust = Ccore * cpc * T0 / V0 * mass_flow_rate + propeller_thrust
    fuel_mass_flow_rate = fuel_to_air_ratio * mass_flow_rate
    specific_fuel_consumption = fuel_mass_flow_rate / thrust if thrust > 0 else np.nan
    power = propeller_thrust * V0
    Tt9 = T9 * (1 + (gamma_t - 1) / 2 * M9 ** 2)
    Pt9 = Pt9_P0 * P0

    out[4], out[5], out[6], out[7], out[8] = tau_c, pi_c, tau_tL, pi_tL, M9
    out[9], out[10], out[11], out[12], out[13], out[14], out[15] = Pt9_P0, mass_flow_rate, fuel_to_air_ratio, thrust, fuel_mass_flow_rate, specific_fuel_consumption, power
    out[16], out[17], out[18] = iteration + 1, 1.0 if converged else 0.0, abs(tau_tL - tau_tL_prev)
    out[19], out[20], out[21], out[22], out[23] = V9, T9, P9, Tt9, Pt9
    out[25] = shaft_power
    out[26] = 0.0
    return out

def turboprop_result_from_kernel_output(output, max_iterations):
    """Data result (the solve_turboprop_offdesign schema) from a solve_turboprop_offdesign_kernel output array."""
    result = Data()
    for index, field in enumerate(TURBOPROP_KERNEL_OUTPUT_FIELDS[:-1]):
        result[field] = float(output[index])
    result.iterations = int(output[16])
    result.converged  = bool(output[17] == 1.0)
    if output[26] == 1.0:
        result.message = (f"no free-turbine pressure ratio matches the core nozzle mass flow (Pt9/P0={output[9]:.4f} "
                          f"at the last ratio) at iteration {result.iterations - 1}")
    else:
        result.message = '' if result.converged else f"did not converge in {max_iterations} iterations (last delta={output[18]:.2e})"
    return result

# ----------------------------------------------------------------------------------------------------------------------
#  solve_turboprop_offdesign
# ----------------------------------------------------------------------------------------------------------------------
def solve_turboprop_offdesign(design_constants, reference_point, mach_number, static_temperature, static_pressure,
                               combustor_exit_temperature, tolerance=1e-8, max_iterations=200, initial_guess=None,
                               shaft_power_offtake=None):
    """
    Off-design component-matching solve for RCAIDE's `Turboprop` (single-
    spool gas generator -- one compressor, one HP/gas-generator turbine --
    plus a mechanically-decoupled free/power turbine driving a propeller
    through a gearbox, no bypass): iterates the gas-generator spool and the
    free-turbine/core-nozzle mass-flow match to a converged operating point
    at a given flight condition and throttle setting, entirely from the
    engine's design-point reference state -- no compressor/turbine
    performance maps needed.

    Unlike `Turbofan`/`Turbojet`, the free turbine drives no compressor, so
    it has no LP-spool power-balance equation -- its tau_tL/pi_tL come
    purely from mass-flow conservation with the downstream (convergent,
    `Expansion_Nozzle`-typed, not `Turbojet`'s `Supersonic_Nozzle`) core
    nozzle, the same equation `solve_turbofan_offdesign` uses for its own LP
    turbine (Mattingly, Elements of Propulsion, 2006, Eq. 8.52w). Because the
    gas generator does not depend on pi_tL, that equation is solved exactly
    for pi_tL at each iteration by bisection between the ratio that leaves the
    nozzle no pressure to expand and no expansion, rather than by fixed-point
    iteration from the design value: at sea level the gas generator delivers
    less pressure than at the design point, and the design pi_tL would leave
    Pt9/P0 below 1 before the iteration could correct it. The gas-generator spool (compressor + HP turbine) matches
    `solve_turbojet_offdesign`'s single compressor-turbine pairing exactly.

    The core-jet thrust is the work-interaction-coefficient form of
    `Turboprop/compute_thrust.py` (Mattingly/Heiser/Pratt Ref. [1] Appendix
    K Eq. (K.6)). The propeller thrust is that of an actuator disk driven by
    the shaft power the free turbine delivers through the gearbox, with the
    polytropic efficiency calibrated to the propeller's design efficiency
    (Cantwell Ref. [2] Ch. 6 Eqs. (6.7)-(6.18), see
    `compute_actuator_disk_propeller_thrust`) rather than a constant
    propeller efficiency, whose thrust eta*P/V0 is singular at zero speed.

    Parameters
    ----------
    design_constants : Data
        Fixed engine constants -- see the schema comment above this function.
    reference_point : Data
        The engine's design-point flight condition and converged cycle
        state -- see the schema comment above this function.
    mach_number, static_temperature, static_pressure : float
        Freestream Mach number, static temperature [K], static pressure [Pa]
        of the off-design flight condition.
    combustor_exit_temperature : float
        Combustor exit (turbine inlet) stagnation temperature [K] -- the
        throttle setting.
    tolerance, max_iterations : optional
        Convergence tolerance on tau_c and tau_tL between iterations, and the iteration limit.
    initial_guess : tuple of float, optional
        (tau_c, tau_tL, pi_tL) starting guess, used instead of the reference
        point's own values -- for `solve_turboprop_offdesign_robust`'s
        continuation stepping.
    shaft_power_offtake : float, optional
        Net shaft power taken from the gas-generator spool [W] (positive for a generator, negative for a
        motor); None uses the design-point offtake.

    Returns
    -------
    Data
        Never raises -- check `.converged` instead. On success: tau_c, pi_c,
        tau_tL, pi_tL, M9, mass_flow_rate [kg/s], fuel_to_air_ratio, thrust
        [N], fuel_mass_flow_rate [kg/s], specific_fuel_consumption
        [kg/(N.s)], power [W] (propeller thrust power: propeller thrust times
        flight speed), shaft_power [W] (power delivered to the propeller
        shaft: free-turbine work after the mechanical and gearbox losses, the
        quantity a turboprop shaft-power rating is quoted in),
        core_nozzle_exit_velocity, converged,
        convergence_delta, iterations, eta_c_used, message=''. On failure:
        converged=False, convergence_delta=inf, message explains why;
        spool-state fields reached before failure are real, everything
        downstream of the nozzle exit state is NaN.

    References
    ----------
    [1] Mattingly, J. D., Heiser, W. H., and Pratt, D. T., "Aircraft Engine
        Design", 2nd ed., AIAA Education Series, 2002, Appendix K
        ("Turboprop Engine Cycle Analysis").
    [2] Cantwell, B., "AA283 Course Notes", Stanford University, Ch. 6 ("The
        Turboprop Cycle"), Secs. 6.2, 6.5.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.solve_turboprop_offdesign_robust
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.design_turboprop_offdesign_matching
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.compute_thrust
    """
    guess  = (np.nan, np.nan, np.nan) if initial_guess is None else initial_guess
    output = solve_turboprop_offdesign_kernel(pack_turboprop_design_constants(design_constants),
                                              pack_turboprop_reference_point(reference_point),
                                              float(mach_number), float(static_temperature), float(static_pressure),
                                              float(combustor_exit_temperature), float(tolerance), int(max_iterations),
                                              float(guess[0]), float(guess[1]), float(guess[2]),
                                              np.nan if shaft_power_offtake is None else float(shaft_power_offtake))
    return turboprop_result_from_kernel_output(output, max_iterations)


# ----------------------------------------------------------------------------------------------------------------------
#  solve_turboprop_offdesign_robust
# ----------------------------------------------------------------------------------------------------------------------
def solve_turboprop_offdesign_robust(design_constants, reference_point, mach_number, static_temperature,
                                      static_pressure, combustor_exit_temperature, tolerance=1e-8, max_iterations=200,
                                      max_continuation_steps=32, allow_unconverged_fallback=False,
                                      packed_design_constants=None, packed_reference_point=None, shaft_power_offtake=None):
    """
    Robust wrapper around `solve_turboprop_offdesign`: tries a direct solve
    first, falls back to continuation if that fails. Same rationale and
    behavior as `Turbofan_OffDesign_Matching.solve_turbofan_offdesign_robust`
    (see its docstring) -- no map-based fallback tier here.

    shaft_power_offtake is passed to `solve_turboprop_offdesign`; continuation steps it from the
    design-point offtake along with the flight condition and combustor exit temperature.

    Returns
    -------
    Data
        Same fields as `solve_turboprop_offdesign`.

    Raises
    ------
    OffDesignMatchingError
        If every tier fails and `allow_unconverged_fallback` is False.
        Also raised, without solving, when the combustor exit temperature is not above the inlet
        stagnation temperature.
    """
    # a combustor exit temperature at or below the inlet stagnation temperature (e.g. zero throttle) has no
    # matched operating point; fail cleanly so callers route the point to their idle_fallback
    inlet_stagnation_temperature = float(static_temperature) * (1 + (design_constants.gamma_c - 1) / 2 * float(mach_number) ** 2)
    if not float(combustor_exit_temperature) > inlet_stagnation_temperature:
        raise OffDesignMatchingError(
            f"turboprop off-design matching: combustor exit temperature {float(combustor_exit_temperature):.1f} K is not above "
            f"the inlet stagnation temperature {inlet_stagnation_temperature:.1f} K (throttle too low for a matched operating point)")
    # direct solve and continuation run in the compiled kernel; only the returned point becomes a Data result
    design    = pack_turboprop_design_constants(design_constants) if packed_design_constants is None else packed_design_constants
    reference = pack_turboprop_reference_point(reference_point)  if packed_reference_point  is None else packed_reference_point
    M0, T0, P0, Tt4 = float(mach_number), float(static_temperature), float(static_pressure), float(combustor_exit_temperature)
    offtake_design = design[16] * reference[9]
    offtake        = offtake_design if shaft_power_offtake is None else float(shaft_power_offtake)
    output = solve_turboprop_offdesign_kernel(design, reference, M0, T0, P0, Tt4, tolerance, max_iterations,
                                              np.nan, np.nan, np.nan, offtake)
    if output[17] == 1.0:
        return turboprop_result_from_kernel_output(output, max_iterations)
    best_partial = output if np.isfinite(output[18]) else None
    n_steps = 2
    while n_steps <= max_continuation_steps:
        tau_c, tau_tL, pi_tL = reference_point.tau_c, reference_point.tau_tL, reference_point.pi_tL
        for k in range(1, n_steps + 1):
            frac   = k / n_steps
            output = solve_turboprop_offdesign_kernel(design, reference,
                                                      reference_point.M0 + frac * (M0 - reference_point.M0),
                                                      reference_point.T0 + frac * (T0 - reference_point.T0),
                                                      reference_point.P0 + frac * (P0 - reference_point.P0),
                                                      reference_point.Tt4 + frac * (Tt4 - reference_point.Tt4),
                                                      tolerance, max_iterations, tau_c, tau_tL, pi_tL,
                                                      offtake_design + frac * (offtake - offtake_design))
            if output[17] != 1.0:
                break
            tau_c, tau_tL, pi_tL = output[4], output[6], output[7]
        if output[17] == 1.0:
            return turboprop_result_from_kernel_output(output, max_iterations)
        if np.isfinite(output[18]) and (best_partial is None or output[18] < best_partial[18]):
            best_partial = output
        n_steps *= 2
    if allow_unconverged_fallback and best_partial is not None:
        return turboprop_result_from_kernel_output(best_partial, max_iterations)
    raise OffDesignMatchingError(
        f"turboprop off-design matching failed even with {max_continuation_steps}-step continuation "
        f"from the reference point (target M0={mach_number}, T0={static_temperature}, "
        f"P0={static_pressure}, Tt4={combustor_exit_temperature})"
    )
