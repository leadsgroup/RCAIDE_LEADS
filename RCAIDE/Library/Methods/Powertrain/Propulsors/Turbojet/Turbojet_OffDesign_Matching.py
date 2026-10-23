# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbojet/Turbojet_OffDesign_Matching.py
#
#
# Created:  Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_OffDesign_Matching import (
    compressor_pressure_ratio, OffDesignMatchingError, njit, constant_efficiency_pressure_ratio_kernel)

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  compiled matching kernel
# ----------------------------------------------------------------------------------------------------------------------
# The same equations as solve_turbojet_offdesign, on plain floats so they compile with numba (see the turbofan
# kernel in Turbofan_OffDesign_Matching). design_constants and reference_point are packed in the orders below.
TURBOJET_DESIGN_CONSTANT_FIELDS = ['gamma_c', 'gamma_t', 'cpc', 'cpt', 'pi_dmax', 'fuel_heating_value', 'eta_b', 'pi_b',
                                   'pi_tH', 'tau_tH', 'pi_n', 'eta_cH', 'eta_c', 'eta_tL', 'shaft_work_specific_design']
TURBOJET_REFERENCE_POINT_FIELDS = ['M0', 'T0', 'P0', 'Tt4', 'tau_c', 'tau_tL', 'pi_tL', 'tau_cH', 'pi_cH', 'pi_c', 'm0']
TURBOJET_KERNEL_OUTPUT_FIELDS   = ['tau_r', 'pi_r', 'pi_d', 'tau_lambda', 'tau_cH', 'pi_cH', 'tau_c', 'pi_c', 'tau_tL',
                                   'pi_tL', 'M9', 'stagnation_to_ambient_core_nozzle_pressure_ratio', 'mass_flow_rate',
                                   'fuel_to_air_ratio', 'thrust', 'fuel_mass_flow_rate', 'specific_fuel_consumption',
                                   'iterations', 'converged', 'convergence_delta', 'core_nozzle_exit_velocity',
                                   'core_nozzle_exit_static_temperature', 'core_nozzle_exit_static_pressure',
                                   'core_nozzle_exit_stagnation_temperature', 'core_nozzle_exit_stagnation_pressure',
                                   'eta_cH_used', 'eta_c_used', 'collapsed']

def pack_turbojet_design_constants(design_constants):
    """Design constants as a float array in TURBOJET_DESIGN_CONSTANT_FIELDS order (for the compiled kernel)."""
    return np.array([float(getattr(design_constants, field, 0.0)) for field in TURBOJET_DESIGN_CONSTANT_FIELDS])

def pack_turbojet_reference_point(reference_point):
    """Reference point as a float array in TURBOJET_REFERENCE_POINT_FIELDS order (for the compiled kernel)."""
    return np.array([float(reference_point[field]) for field in TURBOJET_REFERENCE_POINT_FIELDS])

@njit(cache=True)
def solve_turbojet_offdesign_kernel(design, reference, M0, T0, P0, Tt4, tolerance, max_iterations, relaxation_factor,
                                    guess_tau_c, guess_tau_tL, guess_pi_tL, shaft_power_offtake):
    """
    Compiled solve_turbojet_offdesign: design and reference are the packed design constants and reference
    point, and the result is a float array in TURBOJET_KERNEL_OUTPUT_FIELDS order. A NaN guess starts from
    the reference point, and a NaN shaft_power_offtake uses the design-point offtake.
    """
    gamma_c, gamma_t, cpc, cpt, pi_dmax, fuel_heating_value, eta_b, pi_b = design[0], design[1], design[2], design[3], design[4], design[5], design[6], design[7]
    pi_tH, tau_tH, pi_n, eta_cH, eta_c, eta_tL, shaft_work_specific_design = design[8], design[9], design[10], design[11], design[12], design[13], design[14]
    M0R, T0R, P0R, Tt4R, tau_cR, tau_tLR, pi_tLR = reference[0], reference[1], reference[2], reference[3], reference[4], reference[5], reference[6]
    tau_cHR, pi_cHR, pi_cR, m0R = reference[7], reference[8], reference[9], reference[10]

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
    tau_cH = tau_cHR
    pi_cH = pi_cHR
    pi_c = pi_cR
    Pt9_P0 = 1.0
    tau_tL_prev = tau_tL
    iteration = 0
    for i in range(max_iterations):
        iteration = i
        tau_tL_prev = tau_tL
        tau_c_prev = tau_c
        X  = tau_lambda / (tau_r * tau_c)
        XR = tau_lambdaR / (tau_rR * tau_cR)
        shaft_work_specific = P_offtake / mass_flow_rate_estimate
        phi = shaft_work_specific / (cpt * Tt4)
        tau_cH = 1 + (X / XR) * (tau_cHR - 1) + X * (phiR - phi)
        pi_cH = constant_efficiency_pressure_ratio_kernel(tau_cH, eta_cH, gamma_c)
        pi_c = constant_efficiency_pressure_ratio_kernel(tau_c, eta_c, gamma_c)
        Pt9_P0 = pi_r * pi_d * pi_c * pi_cH * pi_b * pi_tH * pi_tL * pi_n
        if Pt9_P0 < 1.0:
            collapsed = True
            break
        M9 = np.sqrt(2 / (gamma_c - 1) * (Pt9_P0 ** ((gamma_c - 1) / gamma_c) - 1))
        mass_flow_rate_estimate = m0R * (P0 * pi_r * pi_d * pi_c * pi_cH) / \
            (P0R * pi_rR * pi_dR * pi_cR * pi_cHR) * np.sqrt(Tt4R / max(Tt4, 1e-6))
        pi_tL_computed = pi_tLR * np.sqrt(tau_tL / tau_tLR)
        pi_tL = pi_tL + relaxation_factor * (pi_tL_computed - pi_tL)
        tau_tL = 1 - eta_tL * (1 - pi_tL ** ((gamma_t - 1) / gamma_t))
        tau_c_computed = 1 + ((1 - tau_tL) / (1 - tau_tLR)) * ((tau_lambda / tau_r) / (tau_lambdaR / tau_rR)) * \
            (tau_cR - 1)
        tau_c = tau_c + relaxation_factor * (tau_c_computed - tau_c)
        if i > 0 and abs(tau_tL - tau_tL_prev) < tolerance and abs(tau_c - tau_c_prev) < tolerance:
            converged = True
            break

    out = np.full(28, np.nan)
    out[0], out[1], out[2], out[3] = tau_r, pi_r, pi_d, tau_lambda
    out[4], out[5], out[6], out[7], out[8], out[9] = tau_cH, pi_cH, tau_c, pi_c, tau_tL, pi_tL
    out[25], out[26] = eta_cH, eta_c
    if collapsed:
        out[11] = Pt9_P0
        out[17], out[18], out[19] = iteration + 1, 0.0, np.inf
        out[27] = 1.0
        return out

    X  = tau_lambda / (tau_r * tau_c)
    XR = tau_lambdaR / (tau_rR * tau_cR)
    shaft_work_specific = P_offtake / mass_flow_rate_estimate
    phi = shaft_work_specific / (cpt * Tt4)
    tau_cH = 1 + (X / XR) * (tau_cHR - 1) + X * (phiR - phi)
    pi_cH = constant_efficiency_pressure_ratio_kernel(tau_cH, eta_cH, gamma_c)
    pi_c = constant_efficiency_pressure_ratio_kernel(tau_c, eta_c, gamma_c)
    Pt9_P0 = pi_r * pi_d * pi_c * pi_cH * pi_b * pi_tH * pi_tL * pi_n
    M9 = np.sqrt(2 / (gamma_c - 1) * (Pt9_P0 ** ((gamma_c - 1) / gamma_c) - 1))
    mass_flow_rate = m0R * (P0 * pi_r * pi_d * pi_c * pi_cH) / (P0R * pi_rR * pi_dR * pi_cR * pi_cHR) * \
        np.sqrt(Tt4R / max(Tt4, 1e-6))
    tau_x = tau_r * tau_c * tau_cH
    fuel_to_air_ratio = (tau_lambda - tau_x) / (fuel_heating_value * eta_b / (cpc * T0) - tau_lambda)
    Tt9 = Tt4 * tau_tH * tau_tL
    P9 = P0
    T9 = Tt9 / (1 + (gamma_c - 1) / 2 * M9 ** 2)
    a9 = np.sqrt(gamma_c * Rc * T9)
    V9 = M9 * a9
    thrust = mass_flow_rate * ((1 + fuel_to_air_ratio) * V9 - V0)
    fuel_mass_flow_rate = fuel_to_air_ratio * mass_flow_rate
    specific_fuel_consumption = fuel_mass_flow_rate / thrust if thrust > 0 else np.nan
    Pt9 = Pt9_P0 * P0

    out[4], out[5], out[6], out[7], out[8], out[9] = tau_cH, pi_cH, tau_c, pi_c, tau_tL, pi_tL
    out[10], out[11], out[12], out[13] = M9, Pt9_P0, mass_flow_rate, fuel_to_air_ratio
    out[14], out[15], out[16] = thrust, fuel_mass_flow_rate, specific_fuel_consumption
    out[17], out[18], out[19] = iteration + 1, 1.0 if converged else 0.0, abs(tau_tL - tau_tL_prev)
    out[20], out[21], out[22], out[23], out[24] = V9, T9, P9, Tt9, Pt9
    out[27] = 0.0
    return out

def turbojet_result_from_kernel_output(output, max_iterations):
    """Data result (the solve_turbojet_offdesign schema) from a solve_turbojet_offdesign_kernel output array."""
    result = Data()
    for index, field in enumerate(TURBOJET_KERNEL_OUTPUT_FIELDS[:-1]):
        result[field] = float(output[index])
    result.iterations = int(output[17])
    result.converged  = bool(output[18] == 1.0)
    if output[27] == 1.0:
        result.message = f"core nozzle collapsed (Pt9/P0={output[11]:.4f} < 1) at iteration {result.iterations - 1}"
    else:
        result.message = '' if result.converged else f"did not converge in {max_iterations} iterations (last delta={output[19]:.2e})"
    return result

# ----------------------------------------------------------------------------------------------------------------------
#  solve_turbojet_offdesign
# ----------------------------------------------------------------------------------------------------------------------
def solve_turbojet_offdesign(design_constants, reference_point, mach_number, static_temperature, static_pressure,
                              combustor_exit_temperature, tolerance=1e-8, max_iterations=200, relaxation_factor=0.5,
                              initial_guess=None, shaft_power_offtake=None):
    """
    Off-design component-matching solve for RCAIDE's `Turbojet` (two spools,
    LP+HP compressor, no bypass) with a fully-expanded convergent-divergent
    core nozzle: iterates both spools to a converged operating point at a
    given flight condition and throttle, entirely from the engine's design-
    point reference state -- no compressor/turbine performance maps needed.

    A dedicated solver, not a bypass=0 reuse of `Turbofan_OffDesign_Matching.
    solve_turbofan_offdesign`: a turbojet's core nozzle is convergent-
    *divergent* (`Supersonic_Nozzle`, fully expanded downstream of an
    always-choked throat), materially different from a turbofan's plain
    convergent `Expansion_Nozzle` -- confirmed against Cantwell Ref. [2]
    Sec. 4.2 and Mattingly Ref. [1] Table 8.4 before implementing. Reference-
    point reproduction is ~4-6% across the tested pressure-ratio range. Full
    derivation history (including two `Supersonic_Nozzle` bugs found along
    the way) in `RESEARCH/22_ATI/Engine_Validation/ENGINE_MODEL_NOTES.md`
    Sec. 14.

    Parameters
    ----------
    design_constants : Data
        Fixed engine constants (design pressure ratios, efficiencies, gas
        properties), from `design_turbojet_offdesign_matching` -- see the
        schema comment above this function for the full field list.
    reference_point : Data
        The engine's design-point flight condition and converged cycle state,
        from `design_turbojet_offdesign_matching`. Every off-design quantity
        below is computed as a ratio relative to this point -- see the schema
        comment above this function for the full field list.
    mach_number : float
        Freestream Mach number of the off-design flight condition.
    static_temperature : float
        Freestream static temperature [K] of the off-design flight condition.
    static_pressure : float
        Freestream static pressure [Pa] of the off-design flight condition.
    combustor_exit_temperature : float
        Combustor exit (turbine inlet) stagnation temperature [K] -- the
        throttle setting.
    tolerance : float, optional
        Convergence tolerance on successive iterates of tau_c and tau_tL.
    max_iterations : int, optional
    relaxation_factor : float, optional
        Under-relaxes the two independently-iterated unknowns (tau_c, pi_tL --
        tau_tL is a slave of pi_tL each pass) as new = old +
        relaxation_factor*(computed - old); 1.0 recovers the raw functional
        iteration. Same rationale as the turbofan solver's own
        `relaxation_factor` (see its docstring) -- limit-cycling, not wrong
        equations.
    initial_guess : tuple of float, optional
        (tau_c, tau_tL, pi_tL) starting guess, used instead of the reference
        point's own values -- for `solve_turbojet_offdesign_robust`'s
        continuation stepping. Same rationale as the turbofan solver's own
        `initial_guess` (see its docstring).
    shaft_power_offtake : float, optional
        Net shaft power taken from the gas-generator spool [W] (positive for a generator, negative for a
        motor); None uses the design-point offtake.

    Returns
    -------
    Data
        Never raises -- check `.converged` instead (see Notes). On success:
        tau_c, pi_c, tau_cH, pi_cH, tau_tL, pi_tL, M9 (core nozzle exit Mach
        number, may exceed 1 -- fully expanded), mass_flow_rate [kg/s],
        fuel_to_air_ratio, thrust [N], fuel_mass_flow_rate [kg/s],
        specific_fuel_consumption [kg/(N.s)], core_nozzle_exit_velocity,
        converged=True, convergence_delta, iterations, eta_c_used,
        eta_cH_used, message=''. On failure: converged=False,
        convergence_delta=inf, message explains why; spool-state fields
        reached before failure are real, everything downstream of the
        nozzle exit state is NaN.

    Notes
    -----
    No compressor/turbine performance maps needed: efficiencies are held at
    their design values, and the operating point comes from scaling
    reference pressure/temperature ratios with choked-flow and power-balance
    relations -- same approach as the turbofan solver, minus bypass/fan-
    nozzle machinery this architecture doesn't have.

    The LP-turbine/LP-compressor pressure-ratio match has no mass-flow-
    parameter ratio at the nozzle exit at all (unlike the turbofan solver):
    a convergent-divergent nozzle's throat is choked essentially always
    (Cantwell Ref. [2] Sec. 4.4, Mattingly Ref. [1] Sec. 8.3), so that ratio
    is exactly 1 at any operating point and drops out, leaving only the
    `sqrt(tau_tL/ref.tau_tL)` scaling below.

    Because the nozzle is fully expanded (P9=P0 always), the thrust equation
    carries no pressure term: `thrust = mass_flow_rate*((1+f)*V9 - V0)` is
    exact, unlike the turbofan solver's core/fan terms.

    References
    ----------
    [1] Mattingly, J. D., "Elements of Gas Turbine Propulsion", 2nd ed., AIAA
        Education Series, 2005, Sec. 8.3 ("Turbojet Engine").
    [2] Cantwell, B., "AA283 Course Notes", Stanford University, Ch. 4 ("The
        Turbojet Cycle"), Secs. 4.2, 4.4.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.solve_turbojet_offdesign_robust
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.design_turbojet_offdesign_matching
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_OffDesign_Matching
    """
    guess  = (np.nan, np.nan, np.nan) if initial_guess is None else initial_guess
    output = solve_turbojet_offdesign_kernel(pack_turbojet_design_constants(design_constants),
                                             pack_turbojet_reference_point(reference_point),
                                             float(mach_number), float(static_temperature), float(static_pressure),
                                             float(combustor_exit_temperature), float(tolerance), int(max_iterations),
                                             float(relaxation_factor), float(guess[0]), float(guess[1]), float(guess[2]),
                                             np.nan if shaft_power_offtake is None else float(shaft_power_offtake))
    return turbojet_result_from_kernel_output(output, max_iterations)

# ----------------------------------------------------------------------------------------------------------------------
#  solve_turbojet_offdesign_robust
# ----------------------------------------------------------------------------------------------------------------------
def solve_turbojet_offdesign_robust(design_constants, reference_point, mach_number, static_temperature,
                                     static_pressure, combustor_exit_temperature, tolerance=1e-8, max_iterations=200,
                                     relaxation_factor=0.5, max_continuation_steps=32, allow_unconverged_fallback=False,
                                     packed_design_constants=None, packed_reference_point=None, shaft_power_offtake=None):
    """
    Robust wrapper around `solve_turbojet_offdesign`: tries a direct solve
    first, falls back to *continuation* if that fails (stepping from the
    reference condition to the target in increasing increments, chaining
    each step's converged state as the next step's guess). Same rationale
    and behavior as `Turbofan_OffDesign_Matching.solve_turbofan_offdesign_robust`
    (see its docstring) -- no compressor-map fallback tier here (no map
    support built for the turbojet solver).

    Parameters
    ----------
    packed_design_constants, packed_reference_point : numpy.ndarray, optional
        pack_turbojet_design_constants(design_constants) and pack_turbojet_reference_point(reference_point),
        when the caller solves many points of the same engine and has already packed them.
    shaft_power_offtake : float, optional
        Passed to `solve_turbojet_offdesign`; continuation steps it from the design-point offtake along
        with the flight condition and combustor exit temperature.
    allow_unconverged_fallback : bool, optional
        If every tier fails, raises `OffDesignMatchingError` by default. If
        `True`, returns the best (smallest convergence delta) partial result
        seen, with `converged=False`.

    Returns
    -------
    Data
        Same fields as `solve_turbojet_offdesign`.

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
            f"turbojet off-design matching: combustor exit temperature {float(combustor_exit_temperature):.1f} K is not above "
            f"the inlet stagnation temperature {inlet_stagnation_temperature:.1f} K (throttle too low for a matched operating point)")
    # direct solve and continuation run in the compiled kernel; only the returned point becomes a Data result
    design    = pack_turbojet_design_constants(design_constants) if packed_design_constants is None else packed_design_constants
    reference = pack_turbojet_reference_point(reference_point)  if packed_reference_point  is None else packed_reference_point
    M0, T0, P0, Tt4 = float(mach_number), float(static_temperature), float(static_pressure), float(combustor_exit_temperature)
    offtake_design = getattr(design_constants, 'shaft_work_specific_design', 0.0) * reference_point.m0
    offtake        = offtake_design if shaft_power_offtake is None else float(shaft_power_offtake)
    output = solve_turbojet_offdesign_kernel(design, reference, M0, T0, P0, Tt4, tolerance, max_iterations,
                                             relaxation_factor, np.nan, np.nan, np.nan, offtake)
    if output[18] == 1.0:
        return turbojet_result_from_kernel_output(output, max_iterations)
    best_partial = output if np.isfinite(output[19]) else None
    n_steps = 2
    while n_steps <= max_continuation_steps:
        tau_c, tau_tL, pi_tL = reference_point.tau_c, reference_point.tau_tL, reference_point.pi_tL
        for k in range(1, n_steps + 1):
            frac   = k / n_steps
            output = solve_turbojet_offdesign_kernel(design, reference,
                                                     reference_point.M0 + frac * (M0 - reference_point.M0),
                                                     reference_point.T0 + frac * (T0 - reference_point.T0),
                                                     reference_point.P0 + frac * (P0 - reference_point.P0),
                                                     reference_point.Tt4 + frac * (Tt4 - reference_point.Tt4),
                                                     tolerance, max_iterations, relaxation_factor, tau_c, tau_tL, pi_tL,
                                                     offtake_design + frac * (offtake - offtake_design))
            if output[18] != 1.0:
                break
            tau_c, tau_tL, pi_tL = output[6], output[8], output[9]
        if output[18] == 1.0:
            return turbojet_result_from_kernel_output(output, max_iterations)
        if np.isfinite(output[19]) and (best_partial is None or output[19] < best_partial[19]):
            best_partial = output
        n_steps *= 2
    if allow_unconverged_fallback and best_partial is not None:
        return turbojet_result_from_kernel_output(best_partial, max_iterations)
    raise OffDesignMatchingError(
        f"turbojet off-design matching failed even with {max_continuation_steps}-step continuation "
        f"from the reference point (target M0={mach_number}, T0={static_temperature}, "
        f"P0={static_pressure}, Tt4={combustor_exit_temperature})"
    )

# ----------------------------------------------------------------------------------------------------------------------
#  apply_turbojet_afterburner
# ----------------------------------------------------------------------------------------------------------------------
def apply_turbojet_afterburner(result, design_constants, afterburner, working_fluid, static_temperature, static_pressure):
    """
    Adds afterburner (reheat) operation to a converged dry off-design turbojet solution.

    Parameters
    ----------
    result : Data
        Converged solve_turbojet_offdesign / solve_turbojet_offdesign_robust result (dry).
    design_constants : Data
        The engine's design constants (see design_turbojet_offdesign_matching).
    afterburner : RCAIDE.Library.Components.Powertrain.Converters.Combustor
        Afterburner with turbine_inlet_temperature (afterburner exit stagnation temperature Tt7 [K]),
        pressure_ratio, efficiency and fuel_data.specific_energy.
    working_fluid : RCAIDE.Library.Attributes.Gases
        Gas model used for the afterburner gas properties at Tt7.
    static_temperature, static_pressure : float
        Freestream static temperature [K] and pressure [Pa].

    Returns
    -------
    result : Data
        The same result with the nozzle exit state, thrust, fuel flow and fuel-to-air ratio for
        afterburner operation (unchanged if Tt7 does not exceed the turbine exit temperature).

    Notes
    -----
    With the afterburner lit, the variable-area exhaust nozzle is scheduled so that the turbine exit
    conditions are unchanged, so the dry gas-generator solution (pi_tL, tau_tL, mass flow, main
    burner fuel-to-air ratio) is retained and only the afterburner and nozzle are added,
    Ref. [1] Sec. 8-4, Eqs. (8-39) and (8-45r)-(8-45z). The exhaust is fully expanded (P9 = P0), as
    in the dry solution. Afterburner gas properties are those of the working fluid at Tt7.

    References
    ----------
    [1] Mattingly, J. D., "Elements of Gas Turbine Propulsion", McGraw-Hill, 1996, Sec. 8-4.
    """
    Tt7 = afterburner.turbine_inlet_temperature
    T0, P0 = static_temperature, static_pressure
    Tt5 = result.tau_lambda * design_constants.tau_tH * result.tau_tL * design_constants.cpc * T0 / design_constants.cpt
    if not (Tt7 > Tt5) or not np.isfinite(result.thrust):
        return result

    dc         = design_constants
    gamma_c    = dc.gamma_c
    Rc         = (gamma_c - 1) / gamma_c * dc.cpc
    V0         = np.sqrt(gamma_c * Rc * T0) * np.sqrt(2 / (gamma_c - 1) * (result.tau_r - 1))
    cp_AB      = float(np.ravel(working_fluid.compute_cp(Tt7, P0))[0])
    gamma_AB   = float(np.ravel(working_fluid.compute_gamma(Tt7, P0))[0])
    R_AB       = (gamma_AB - 1) / gamma_AB * cp_AB                                                   # (8-45r)
    tau_lambda_AB = cp_AB * Tt7 / (dc.cpc * T0)                                                      # (8-45s)
    f_AB = (tau_lambda_AB - result.tau_lambda * dc.tau_tH * result.tau_tL) / \
           (afterburner.fuel_data.specific_energy * afterburner.efficiency / (dc.cpc * T0) - tau_lambda_AB)  # (8-45t)
    Pt9_P9 = result.stagnation_to_ambient_core_nozzle_pressure_ratio * afterburner.pressure_ratio    # (8-45u), P9 = P0
    M9     = np.sqrt(2 / (gamma_AB - 1) * (Pt9_P9 ** ((gamma_AB - 1) / gamma_AB) - 1))              # (8-45v)
    T9     = Tt7 / (Pt9_P9 ** ((gamma_AB - 1) / gamma_AB))                                          # (8-45w)
    V9     = M9 * np.sqrt(gamma_AB * R_AB * T9)                                                      # (8-45x)
    f_0    = result.fuel_to_air_ratio + f_AB                                                         # (8-45y)
    thrust = result.mass_flow_rate * ((1 + f_0) * V9 - V0)                                           # (8-45z), P9 = P0

    result.stagnation_to_ambient_core_nozzle_pressure_ratio = Pt9_P9
    result.M9                                    = M9
    result.fuel_to_air_ratio                     = f_0
    result.thrust                                = thrust
    result.fuel_mass_flow_rate                   = f_0 * result.mass_flow_rate
    result.specific_fuel_consumption             = result.fuel_mass_flow_rate / thrust if thrust > 0 else np.nan
    result.core_nozzle_exit_velocity             = V9
    result.core_nozzle_exit_static_temperature   = T9
    result.core_nozzle_exit_static_pressure      = P0
    result.core_nozzle_exit_stagnation_temperature = Tt7
    result.core_nozzle_exit_stagnation_pressure  = Pt9_P9 * P0
    return result
