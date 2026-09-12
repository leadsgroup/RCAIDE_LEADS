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
    compressor_pressure_ratio, OffDesignMatchingError)

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  solve_turbojet_offdesign
# ----------------------------------------------------------------------------------------------------------------------
def solve_turbojet_offdesign(design_constants, reference_point, mach_number, static_temperature, static_pressure,
                              combustor_exit_temperature, tolerance=1e-8, max_iterations=200, relaxation_factor=0.5,
                              initial_guess=None):
    """
    Off-design component-matching solve for a single-spool-equivalent
    turbojet (RCAIDE's `Turbojet`: two spools, LP compressor + HP compressor,
    no bypass) with a fully-expanded convergent-divergent core nozzle:
    iterates the LP-turbine/LP-compressor and HP-compressor/HP-turbine
    spools to a converged operating point at a given flight condition and
    throttle setting, entirely from the engine's design-point reference
    state -- no compressor/turbine performance maps needed (see Notes).

    A dedicated turbojet solver, not a reuse of `Turbofan_OffDesign_Matching.
    solve_turbofan_offdesign` with bypass pinned to zero: a turbojet's core
    nozzle is convergent-*divergent* (RCAIDE's `Supersonic_Nozzle`), fully
    expanding downstream of an always-choked throat -- a materially
    different nozzle model from a turbofan's plain convergent
    `Expansion_Nozzle`. Confirmed against Cantwell Ref. [2] Sec. 4.2 and
    Mattingly Ref. [1] Table 8.4 before implementing. Reference-point
    reproduction is a bounded ~4-6% across the tested pressure-ratio range
    (subsonic through GE90-class PR), comparable to the turbofan solver's
    own ~2% baseline -- see `RESEARCH/22_ATI/Engine_Validation/
    ENGINE_MODEL_NOTES.md` Sec. 14 for the full derivation history,
    including two `Supersonic_Nozzle` bugs found along the way (one matched
    rather than fixed -- freestream vs. local gas properties for exit Mach,
    out of scope to change everywhere else it's used; one fixed directly --
    exit static pressure left at a placeholder when choked).

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
    No compressor/turbine performance MAPS are required: component
    efficiencies are held at their design (reference) values, and the
    operating point is found by scaling reference pressure/temperature
    ratios with choked-flow and power-balance relations, generalized to the
    LP-compressor+LP-turbine / HP-compressor+HP-turbine spools -- same
    approach as the turbofan solver, minus the bypass/fan-nozzle machinery
    that engine architecture doesn't have.

    The LP-turbine/LP-compressor spool's pressure-ratio matching equation is
    *simpler* than the turbofan solver's equivalent (no mass-flow-parameter
    ratio at the nozzle exit at all): the turbofan version matches mass flow
    at the nozzle's own choked throat, which for a plain convergent nozzle
    *is* the exit plane (so the exit Mach, capped at 1 when choked, appears
    directly in that ratio). A convergent-divergent nozzle's throat is a
    physically different, upstream station from its exit -- fixed-area and,
    per both Cantwell Ref. [2] Sec. 4.4 ("the nozzle throat is also choked...
    over almost the entire practical range of engine operating conditions")
    and Mattingly Ref. [1] Sec. 8.3 (same assumption for the single-spool
    gas generator), choked (M=1) essentially always. A fixed area choked at
    M=1 at both the reference and current operating point makes that ratio
    exactly 1 regardless of what the diverging section does further
    downstream -- it drops out of the equation entirely, leaving only the
    `sqrt(tau_tL/ref.tau_tL)` temperature-ratio scaling below.

    Because the nozzle is fully expanded (P9=P0 always, by the same Ref. [2]
    Sec. 4.2 assumption used throughout this solver), the thrust equation
    carries no pressure term at all (unlike the turbofan solver's core/fan
    nozzle terms, which track a real choked-nozzle pressure mismatch) --
    `thrust = mass_flow_rate*((1+f)*V9 - V0)` is exact, not an approximation
    that happens to drop a small correction.

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
    dc, ref = design_constants, reference_point
    M0, T0, P0, Tt4 = mach_number, static_temperature, static_pressure, combustor_exit_temperature

    gamma_c = dc.gamma_c
    gamma_t = dc.gamma_t
    Rc = (gamma_c - 1) / gamma_c * dc.cpc
    Rt = (gamma_t - 1) / gamma_t * dc.cpt

    a0 = np.sqrt(gamma_c * Rc * T0)
    V0 = a0 * M0
    tau_r = 1 + (gamma_c - 1) / 2 * M0 ** 2
    pi_r = tau_r ** (gamma_c / (gamma_c - 1))
    eta_r = 1.0 if M0 <= 1 else 1 - 0.075 * (M0 - 1) ** 1.35
    pi_d = dc.pi_dmax * eta_r
    tau_lambda = dc.cpt * Tt4 / (dc.cpc * T0)

    # reference-point derived quantities, evaluated at the design flight condition
    tau_rR = 1 + (gamma_c - 1) / 2 * ref.M0 ** 2
    pi_rR = tau_rR ** (gamma_c / (gamma_c - 1))
    eta_rR = 1.0 if ref.M0 <= 1 else 1 - 0.075 * (ref.M0 - 1) ** 1.35
    pi_dR = dc.pi_dmax * eta_rR
    tau_lambdaR = dc.cpt * ref.Tt4 / (dc.cpc * ref.T0)

    if initial_guess is None:
        tau_c, tau_tL, pi_tL = ref.tau_c, ref.tau_tL, ref.pi_tL
    else:
        tau_c, tau_tL, pi_tL = initial_guess

    # Shaft power offtake (IDG/motor), same derivation as solve_turbofan_offdesign's own
    # tau_cH extension -- see its schema comment for the full derivation. Zero for an engine
    # with no offtake, reducing this exactly to the original no-offtake equation.
    shaft_work_specific_design = getattr(dc, 'shaft_work_specific_design', 0.0)
    P_offtake_design = shaft_work_specific_design * ref.m0
    phiR = shaft_work_specific_design / (dc.cpt * ref.Tt4)

    mass_flow_rate_estimate = ref.m0
    converged = False
    collapsed = False
    for i in range(max_iterations):
        tau_tL_prev = tau_tL
        tau_c_prev = tau_c

        # HP compressor temperature ratio, from the HP-spool power balance (same form as
        # solve_turbofan_offdesign's tau_cH)
        X  = tau_lambda / (tau_r * tau_c)
        XR = tau_lambdaR / (tau_rR * ref.tau_c)
        shaft_work_specific = P_offtake_design / mass_flow_rate_estimate
        phi = shaft_work_specific / (dc.cpt * Tt4)
        tau_cH = 1 + (X / XR) * (ref.tau_cH - 1) + X * (phiR - phi)
        pi_cH, eta_cH_used = compressor_pressure_ratio(tau_cH, dc.eta_cH, gamma_c)
        pi_c, eta_c_used = compressor_pressure_ratio(tau_c, dc.eta_c, gamma_c)

        # core nozzle -- fully expanded (P9=P0 always, see Notes): M9 solved directly from
        # Pt9/P0, no choked-at-1 cap. Floored at 0 (not raised) for a bad intermediate
        # iterate that pushes Pt9/P0 below 1 -- same "fail cleanly downstream" rationale as
        # compressor_pressure_ratio's own floor (imported from the turbofan module)
        Pt9_P0 = pi_r * pi_d * pi_c * pi_cH * dc.pi_b * dc.pi_tH * pi_tL * dc.pi_n
        if Pt9_P0 < 1.0:
            collapsed = True
            break
        # gamma_c (cold/freestream), not gamma_t -- matches RCAIDE's own Supersonic_Nozzle
        # (compute_supersonic_nozzle_performance.py), which computes exit Mach using
        # conditions.freestream.isentropic_expansion_factor regardless of the actual hot
        # exhaust gas properties at the nozzle inlet (unlike Expansion_Nozzle, which
        # correctly uses the local working-fluid state -- a separate, real bug in
        # Supersonic_Nozzle, not fixed here; see design_turbojet_offdesign_matching's
        # Notes). Matching RCAIDE's actual (if imperfect) behavior here is what makes
        # reference_point.F reproduce turbojet.design_thrust.
        M9 = np.sqrt(2 / (gamma_c - 1) * (Pt9_P0 ** ((gamma_c - 1) / gamma_c) - 1))

        # carried forward for next pass's offtake term above, lagged like pi_tL below
        mass_flow_rate_estimate = ref.m0 * (P0 * pi_r * pi_d * pi_c * pi_cH) / \
            (ref.P0 * pi_rR * pi_dR * ref.pi_c * ref.pi_cH) * np.sqrt(ref.Tt4 / max(Tt4, 1e-6))

        # LP turbine pressure ratio -- see Notes for why this has no mass-flow-parameter
        # ratio at all (the nozzle *throat*, not the fully-expanded exit, sets this
        # matching, and the throat is always choked at both the reference and current
        # point) -- under-relaxed, same rationale as the turbofan solver
        pi_tL_computed = ref.pi_tL * np.sqrt(tau_tL / ref.tau_tL)
        pi_tL = pi_tL + relaxation_factor * (pi_tL_computed - pi_tL)

        # LP turbine temperature ratio from the current (pre-update) pi_tL
        tau_tL = 1 - dc.eta_tL * (1 - pi_tL ** ((gamma_t - 1) / gamma_t))

        # LP compressor temperature ratio, from the LP-spool power balance -- under-relaxed
        tau_c_computed = 1 + ((1 - tau_tL) / (1 - ref.tau_tL)) * ((tau_lambda / tau_r) / (tau_lambdaR / tau_rR)) * \
            (ref.tau_c - 1)
        tau_c = tau_c + relaxation_factor * (tau_c_computed - tau_c)

        if i > 0 and abs(tau_tL - tau_tL_prev) < tolerance and abs(tau_c - tau_c_prev) < tolerance:
            converged = True
            break

    if collapsed:
        result = Data(
            tau_r=tau_r,pi_r=pi_r, pi_d=pi_d, tau_lambda=tau_lambda,
            tau_cH=tau_cH, pi_cH=pi_cH, tau_c=tau_c, pi_c=pi_c,
            tau_tL=tau_tL, pi_tL=pi_tL, M9=np.nan,
            stagnation_to_ambient_core_nozzle_pressure_ratio=Pt9_P0,
            mass_flow_rate=np.nan, fuel_to_air_ratio=np.nan,
            thrust=np.nan, fuel_mass_flow_rate=np.nan, specific_fuel_consumption=np.nan,
            iterations=i + 1, converged=False, convergence_delta=np.inf,
            core_nozzle_exit_velocity=np.nan,
            core_nozzle_exit_static_temperature=np.nan, core_nozzle_exit_static_pressure=np.nan,
            core_nozzle_exit_stagnation_temperature=np.nan, core_nozzle_exit_stagnation_pressure=np.nan,
            eta_cH_used=eta_cH_used, eta_c_used=eta_c_used,
            message=f"core nozzle collapsed (Pt9/P0={Pt9_P0:.4f} < 1) at iteration {i}",
        )
    else:
        # Recompute final-state dependent quantities at whatever tau_tL/tau_c/pi_tL the
        # iteration reached -- converged fixed point on success, or the last iterate if
        # max_iterations ran out
        tau_cH = 1 + ((tau_lambda / tau_r) / (tau_lambdaR / tau_rR)) * (ref.tau_c / tau_c) * (ref.tau_cH - 1)
        pi_cH, eta_cH_used = compressor_pressure_ratio(tau_cH, dc.eta_cH, gamma_c)
        pi_c, eta_c_used = compressor_pressure_ratio(tau_c, dc.eta_c, gamma_c)
        Pt9_P0 = pi_r * pi_d * pi_c * pi_cH * dc.pi_b * dc.pi_tH * pi_tL * dc.pi_n
        M9 = np.sqrt(2 / (gamma_c - 1) * (Pt9_P0 ** ((gamma_c - 1) / gamma_c) - 1))

        # engine mass flow rate -- same station-4 choked-flow scaling as the turbofan
        # solver's mass_flow_rate, minus the (1+alpha)/(1+ref.alpha) bypass-split factor
        # (always 1 here -- no bypass at all)
        mass_flow_rate = ref.m0 * (P0 * pi_r * pi_d * pi_c * pi_cH) / (ref.P0 * pi_rR * pi_dR * ref.pi_c * ref.pi_cH) * \
            np.sqrt(ref.Tt4 / Tt4)

        # fuel/air ratio
        tau_x = tau_r * tau_c * tau_cH  # compressor-exit / T0 (station 3 temperature ratio)
        fuel_to_air_ratio = (tau_lambda - tau_x) / (dc.fuel_heating_value * dc.eta_b / (dc.cpc * T0) - tau_lambda)

        # core nozzle exit state -- fully expanded, so P9=P0 exactly (no separate static-
        # pressure computation needed). Tt9 (stagnation temperature entering the nozzle)
        # is tracked physically via the hot-side energy balance through the turbines
        # (Tt4*tau_tH*tau_tL, cpt-based, correct); T9/a9/V9 downstream of that use gamma_c/
        # Rc, matching RCAIDE's own Supersonic_Nozzle convention (see the M9 comment above)
        Tt9 = Tt4 * dc.tau_tH * tau_tL
        P9 = P0
        T9 = Tt9 / (1 + (gamma_c - 1) / 2 * M9 ** 2)
        a9 = np.sqrt(gamma_c * Rc * T9)
        V9 = M9 * a9

        thrust = mass_flow_rate * ((1 + fuel_to_air_ratio) * V9 - V0)

        fuel_mass_flow_rate = fuel_to_air_ratio * mass_flow_rate
        specific_fuel_consumption = fuel_mass_flow_rate / thrust  # kg/(N.s)

        Pt9 = Pt9_P0 * P0

        convergence_delta = abs(tau_tL - tau_tL_prev)
        result = Data(
            tau_r=tau_r, pi_r=pi_r, pi_d=pi_d, tau_lambda=tau_lambda,
            tau_cH=tau_cH, pi_cH=pi_cH, tau_c=tau_c, pi_c=pi_c,
            tau_tL=tau_tL, pi_tL=pi_tL, M9=M9,
            stagnation_to_ambient_core_nozzle_pressure_ratio=Pt9_P0,
            mass_flow_rate=mass_flow_rate, fuel_to_air_ratio=fuel_to_air_ratio,
            thrust=thrust, fuel_mass_flow_rate=fuel_mass_flow_rate,
            specific_fuel_consumption=specific_fuel_consumption,
            iterations=i + 1, converged=converged, convergence_delta=convergence_delta,
            core_nozzle_exit_velocity=V9,
            core_nozzle_exit_static_temperature=T9, core_nozzle_exit_static_pressure=P9,
            core_nozzle_exit_stagnation_temperature=Tt9, core_nozzle_exit_stagnation_pressure=Pt9,
            eta_cH_used=eta_cH_used, eta_c_used=eta_c_used,
            message='' if converged else f"did not converge in {max_iterations} iterations "
                                          f"(last delta={convergence_delta:.2e})",
        )

    return result


# ----------------------------------------------------------------------------------------------------------------------
#  solve_turbojet_offdesign_robust
# ----------------------------------------------------------------------------------------------------------------------
def solve_turbojet_offdesign_robust(design_constants, reference_point, mach_number, static_temperature,
                                     static_pressure, combustor_exit_temperature, tolerance=1e-8, max_iterations=200,
                                     relaxation_factor=0.5, max_continuation_steps=32, allow_unconverged_fallback=False):
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
    """
    best_partial = None

    def track_best(result):
        nonlocal best_partial
        if not np.isfinite(result.convergence_delta):
            return
        if best_partial is None or result.convergence_delta < best_partial.convergence_delta:
            best_partial = result

    def continuation():
        n_steps = 2
        while n_steps <= max_continuation_steps:
            state = (reference_point.tau_c, reference_point.tau_tL, reference_point.pi_tL)
            result = None
            for k in range(1, n_steps + 1):
                frac = k / n_steps
                M0_k = reference_point.M0 + frac * (mach_number - reference_point.M0)
                T0_k = reference_point.T0 + frac * (static_temperature - reference_point.T0)
                P0_k = reference_point.P0 + frac * (static_pressure - reference_point.P0)
                Tt4_k = reference_point.Tt4 + frac * (combustor_exit_temperature - reference_point.Tt4)
                result = solve_turbojet_offdesign(design_constants, reference_point, M0_k, T0_k, P0_k, Tt4_k,
                                                   tolerance=tolerance, max_iterations=max_iterations,
                                                   relaxation_factor=relaxation_factor, initial_guess=state)
                if not result.converged:
                    break
                state = (result.tau_c, result.tau_tL, result.pi_tL)
            if result.converged:
                return result
            track_best(result)
            n_steps *= 2
        return None

    direct = solve_turbojet_offdesign(design_constants, reference_point, mach_number, static_temperature,
                                       static_pressure, combustor_exit_temperature, tolerance=tolerance,
                                       max_iterations=max_iterations, relaxation_factor=relaxation_factor)
    result = direct if direct.converged else None
    if result is None:
        track_best(direct)
        result = continuation()

    if result is None and allow_unconverged_fallback:
        result = best_partial

    if result is None:
        raise OffDesignMatchingError(
            f"turbojet off-design matching failed even with {max_continuation_steps}-step continuation "
            f"from the reference point (target M0={mach_number}, T0={static_temperature}, "
            f"P0={static_pressure}, Tt4={combustor_exit_temperature})"
        )
    return result
