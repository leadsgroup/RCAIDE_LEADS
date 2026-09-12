# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbofan/Turbofan_OffDesign_Matching.py
#
#
# Created:  Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Framework.Core import Data

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  mfp
# ----------------------------------------------------------------------------------------------------------------------
def mfp(mach_number, gamma, gas_constant):
    """
    Mass flow parameter, MFP(M) = sqrt(gamma/R) * M * (1+(gamma-1)/2*M^2)^(-(gamma+1)/(2*(gamma-1))).
    SI units throughout (R in J/(kg.K)); no gc factor, unlike the English-unit
    reference implementation this was ported from, since SI needs no
    lbm/lbf reconciliation constant.

    References
    ----------
    [1] Mattingly, J. D., "Elements of Gas Turbine Propulsion", 2nd ed., AIAA Education
        Series, 2005, Eq. (2.76).
    """
    mfp = np.sqrt(gamma / gas_constant) * mach_number * \
        (1 + (gamma - 1) / 2 * mach_number ** 2) ** (-(gamma + 1) / (2 * (gamma - 1)))
    return mfp


def area_from_mass_flow_rate(mass_flow_rate, static_pressure, static_temperature, gas_constant, gamma, mach_number):
    """Back out nozzle exit area from mass flow rate (needed for the pressure-thrust term)."""
    stagnation_pressure    = static_pressure * (1 + (gamma - 1) / 2 * mach_number ** 2) ** (gamma / (gamma - 1))
    stagnation_temperature = static_temperature * (1 + (gamma - 1) / 2 * mach_number ** 2)
    return mass_flow_rate * np.sqrt(stagnation_temperature) / (stagnation_pressure * mfp(mach_number, gamma, gas_constant))


def compressor_pressure_ratio(temperature_ratio, constant_efficiency, gamma, compressor_map=None):
    """
    Pressure ratio (and the efficiency used to get it) for a compressor stage
    at a given temperature ratio: a constant-adiabatic-efficiency relation if
    `compressor_map` is None, or a map-consistent pressure ratio/efficiency
    (`Generic_Compressor_Map.query_by_temperature_ratio`) otherwise. Shared by
    both the fan and HP-compressor call sites in `solve_turbofan_offdesign` so
    the map-vs-constant-efficiency choice lives in one place, not duplicated
    at each.

    Parameters
    ----------
    temperature_ratio : float
        The stage's adiabatic temperature ratio, already determined by the
        power-balance equations -- unchanged by which efficiency source is used.
    constant_efficiency : float
        Design-point adiabatic efficiency, used directly when `compressor_map`
        is None.
    gamma : float
        Ratio of specific heats of the working fluid through this stage.
    compressor_map : RCAIDE.Library.Methods.Powertrain.Converters.Compressor.Generic_Compressor_Map, optional
        Already scaled to this stage's own design point (`scale_to_design_point`).

    Returns
    -------
    pressure_ratio : float
    efficiency : float
    """
    if compressor_map is None:
        efficiency = constant_efficiency
        # floored: a bad intermediate iterate at deep part-power can push this base negative,
        # and ** silently returns a complex number there instead of raising -- floor it so a
        # doomed iterate fails cleanly downstream (OffDesignMatchingError) instead of crashing
        # opaquely in nozzle_state's `<` comparison
        base           = max(1 + efficiency * (temperature_ratio - 1), 1e-6)
        pressure_ratio = base ** (gamma / (gamma - 1))
    else:
        percent_corrected_speed, pressure_ratio, efficiency = compressor_map.query_by_temperature_ratio(
            temperature_ratio, gamma)
    return pressure_ratio, efficiency


def nozzle_state(stagnation_to_ambient_pressure_ratio, gamma):
    """
    Given Pt/P0 upstream of a convergent nozzle, returns (P/P0, exit Mach number).

    Pt_ratio < 1 means the nozzle's own stagnation pressure has fallen below
    ambient -- an invalid/collapsed state (the same failure mode RCAIDE's
    fixed-pressure-ratio analytical turbofan model silently clips to ambient
    at extreme off-design points, per the LP turbine exit pressure falling
    to/below ambient and the nozzle being unable to produce positive exit
    velocity). Clipped to exit Mach 0 rather than raising here, so the caller
    (`solve_turbofan_offdesign`) can decide how to handle it explicitly
    (it raises `OffDesignMatchingError` downstream instead of failing on an
    opaque division by zero).
    """
    pi_crit = ((gamma + 1) / 2) ** (gamma / (gamma - 1))
    if stagnation_to_ambient_pressure_ratio < 1.0:
        static_to_ambient_pressure_ratio, exit_mach_number = 1.0, 0.0
    elif stagnation_to_ambient_pressure_ratio < pi_crit:
        static_to_ambient_pressure_ratio = 1.0  # unchoked: exit static pressure = ambient
        exit_mach_number = np.sqrt(2 / (gamma - 1) * (stagnation_to_ambient_pressure_ratio ** ((gamma - 1) / gamma) - 1))
    else:
        static_to_ambient_pressure_ratio = stagnation_to_ambient_pressure_ratio / pi_crit  # choked
        exit_mach_number = 1.0
    return static_to_ambient_pressure_ratio, exit_mach_number


# ----------------------------------------------------------------------------------------------------------------------
#  design_constants / reference_point schema (both plain Data objects, not classes --
#  see design_turbofan_offdesign_matching, which is the only place that builds them)
# ----------------------------------------------------------------------------------------------------------------------
# `solve_turbofan_offdesign` takes two plain `Data` objects rather than dedicated classes,
# consistent with the rest of this package's convention that Methods/ holds functions, not
# data-model classes (those belong under Components/). Both are built once per engine by
# `design_turbofan_offdesign_matching`, which is the authoritative reference for exactly
# what each field means and how it's derived -- this comment only lists the fields, for a
# quick reference while reading the solver below.
#
# design_constants (fixed engine constants, all SI units):
#   gamma_c, gamma_t          ratio of specific heats, cold side (compressor/fan) / hot side
#                             (combustor exit through turbines)
#   cpc, cpt                  specific heat, cold/hot side [J/(kg.K)]
#   fuel_heating_value        [J/kg]
#   pi_dmax                   inlet/diffuser max pressure ratio
#   pi_b, pi_n, pi_fn         combustor / core nozzle / fan nozzle pressure ratio
#   tau_tH, pi_tH             HP turbine temperature/pressure ratio -- constant (station 4
#                             always choked)
#   eta_f, eta_cH             fan+LPC (combined LP spool) / HP compressor adiabatic
#                             efficiency, design point
#   eta_b, eta_mH, eta_mL, eta_tL   combustor efficiency, HP/LP spool mechanical
#                             efficiency, LP turbine adiabatic efficiency
#   shaft_work_specific_design   HP-spool external shaft power offtake (IDG/motor), as
#                             *specific* work [J/kg core flow] at the design point -- read
#                             directly off the converged design point's own
#                             hpt_conditions.inputs.external_shaft.work_done (already
#                             specific there, not a raw power -- see
#                             compute_turbofan_performance.py/design_turbofan.py's own
#                             fix for this same unit conversion). Zero for an engine with
#                             no integrated_drive_generator/integrated_drive_motor. A
#                             fixed *absolute* power (e.g. a constant-power generator load)
#                             becomes a *larger* specific-work fraction at part-power as
#                             mass flow drops -- solve_turbofan_offdesign re-derives the
#                             equivalent absolute power once (design_constants.
#                             shaft_work_specific_design * reference_point.m0) and reapplies
#                             it against each iteration's own (evolving) mass flow estimate,
#                             rather than assuming the offtake scales proportionally with
#                             throttle the way the rest of the HP-spool equation does.
#   eta_f_alone, fan_temperature_rise_fraction   fan ALONE (not combined with LPC) design
#                             adiabatic efficiency, and what fraction of the combined
#                             LP-spool temperature *rise* the fan alone accounts for at the
#                             design point. Exist because Mattingly's off-design equations
#                             (Ref. [1] below, Fig. 8.46) model a *single* "fan" as all of
#                             the LP-spool compression, bypass split at that fan's own
#                             exit -- no separate downstream LPC in his simplified
#                             architecture. RCAIDE's real turbofan has both a fan AND an
#                             LPC on the LP shaft, but the BYPASS stream only physically
#                             passes through the fan -- it splits off before the LPC, which
#                             processes core flow only. So tau_f/pi_f (the fan+LPC
#                             *combined* quantities, correct for LP-spool power balance and
#                             mass-flow matching, since the LP turbine drives both
#                             together) are NOT what the fan nozzle/bypass-stream exit
#                             state should use -- using them there overstates the bypass
#                             stream's compression by including a rise (the LPC's) it never
#                             experiences. These two fields let the solver below recover a
#                             fan-alone ratio from the combined tau_f it already solves
#                             for, specifically for the fan nozzle equations, assuming that
#                             split stays constant off-design (both machines share the same
#                             shaft/speed -- a reasonable first-order assumption, not an
#                             exact law). Found necessary, not assumed up front: an earlier
#                             version used the combined ratio in the fan nozzle equations
#                             too, and reproduced the engine's own design-point thrust to
#                             within 0.5% on the core stream but ~4x too high overall --
#                             traced to the fan nozzle velocity alone coming out 32% too
#                             high, enough to matter a great deal for an engine whose fan
#                             nozzle velocity margin over flight speed is already small.
#
# reference_point (design-point flight condition and converged cycle state; every off-
# design solve is scaled relative to this, so it doubles as the initial guess for a direct
# solve; all SI units):
#   M0, T0, P0, Tt4           design Mach number, freestream static temperature [K],
#                             static pressure [Pa], combustor exit stagnation temperature [K]
#   pi_f, pi_cH, tau_f, tau_cH, tau_tL, pi_tL   converged design-point cycle state
#   alpha                     bypass ratio
#   M9, M19                   core / fan nozzle exit Mach number
#   m0                        total engine mass flow rate [kg/s]
#   F                         net thrust [N] -- optional, for validation only
#
# See RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.design_turbofan_offdesign_matching

# ----------------------------------------------------------------------------------------------------------------------
#  OffDesignMatchingError
# ----------------------------------------------------------------------------------------------------------------------
class OffDesignMatchingError(RuntimeError):
    """
    Raised by `solve_turbofan_offdesign_robust` when no tier (direct solve,
    continuation, map-based continuation, or -- if explicitly allowed -- the
    best unconverged partial result) reaches a usable operating point.
    `solve_turbofan_offdesign` itself never raises: every call returns a
    `Data` with `.converged` (and, on failure, `.message` explaining why --
    either a stable limit cycle that didn't converge in `max_iterations`, or
    the core/fan nozzle's own stagnation pressure collapsing to/below ambient,
    an invalid state rather than just a hard-to-reach one). Status-flag
    results rather than an internal exception so the solve itself has no
    control flow that depends on raising -- callers that just want "give me
    an answer or fail loudly" get that from this wrapper; callers that want
    to inspect a non-converged point themselves (e.g. `idle_fallback` in
    `compute_turbofan_performance_offdesign.py`) can call
    `solve_turbofan_offdesign` directly and check `.converged` without a
    try/except.
    """


# ----------------------------------------------------------------------------------------------------------------------
#  solve_turbofan_offdesign
# ----------------------------------------------------------------------------------------------------------------------
def solve_turbofan_offdesign(design_constants, reference_point, mach_number, static_temperature, static_pressure,
                              combustor_exit_temperature, tolerance=1e-8, max_iterations=200, relaxation_factor=0.5,
                              initial_guess=None, fan_map=None, high_pressure_compressor_map=None):
    """
    Off-design component-matching solve for a separate-exhaust, two-spool
    turbofan with convergent nozzles: iterates the fan/LP-turbine and HP-
    compressor/HP-turbine spools to a converged operating point at a given
    flight condition and throttle setting, entirely from the engine's design-
    point reference state -- no compressor/turbine performance maps are
    needed for this (see Notes).

    Parameters
    ----------
    design_constants : Data
        Fixed engine constants (design pressure ratios, efficiencies, gas
        properties), from `design_turbofan_offdesign_matching` -- see the
        schema comment above this function for the full field list.
    reference_point : Data
        The engine's design-point flight condition and converged cycle state,
        from `design_turbofan_offdesign_matching`. Every off-design quantity
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
        Convergence tolerance on successive iterates of tau_f and tau_tL.
    max_iterations : int, optional
    relaxation_factor : float, optional
        Under-relaxes the two independently-iterated unknowns (tau_f, pi_tL --
        tau_tL is a slave of pi_tL each pass) as new = old +
        relaxation_factor*(computed - old); 1.0 recovers the raw functional
        iteration. Needed because the raw iteration can settle into a stable
        limit cycle rather than converging at extreme off-design points --
        classic Gauss-Seidel behavior, not a sign the equations are wrong.
        Damping changes the convergence path, not the fixed point.
    initial_guess : tuple of float, optional
        (tau_f, tau_tL, pi_tL) starting guess, used instead of the reference
        point's own values -- for `solve_turbofan_offdesign_robust`'s
        continuation stepping. The reference point itself still anchors every
        *_R quantity in the physics; only the iteration's starting guess
        changes. A large jump straight from the design point (e.g. throttling
        back at low altitude) can produce a bad enough first-pass estimate
        that the core nozzle's Pt9/P0 falls below 1 (physically invalid)
        before the iteration has taken a single corrective step --
        relaxation can't fix a bad first guess, since the collapse happens
        before any relaxed update is applied. Continuation (stepping through
        intermediate flight conditions, chaining each step's answer as the
        next step's starting guess) fixes this at the source instead of
        patching the symptom.
    fan_map, high_pressure_compressor_map : RCAIDE.Library.Methods.Powertrain.Converters.Compressor.Generic_Compressor_Map, optional
        Already scaled to this engine's actual design point
        (`scale_to_design_point`). If given, replace the constant-adiabatic-
        efficiency pressure-ratio relation for the fan and/or HP compressor
        with a map-consistent pressure ratio and efficiency, at whatever
        corrected speed the map says produces the same temperature ratio the
        power balance already demands. This does not change what temperature
        ratio is required (still purely the power-balance equations,
        unchanged) -- only how the corresponding pressure ratio is obtained.
        `None` (the default) preserves the constant-efficiency behavior
        exactly.

    Returns
    -------
    Data
        Never raises -- check `.converged` instead (see Notes). On success:
        tau_f, pi_f, tau_cH, pi_cH, tau_tL, pi_tL, alpha (bypass ratio), M9,
        M19 (core/fan nozzle exit Mach numbers), mass_flow_rate [kg/s],
        fuel_to_air_ratio, thrust [N], fuel_mass_flow_rate [kg/s],
        specific_fuel_consumption [kg/(N.s)], core_nozzle_exit_velocity,
        fan_nozzle_exit_velocity [m/s], converged=True, convergence_delta,
        iterations, eta_f_used, eta_cH_used (whichever efficiency source --
        constant or map -- actually produced pi_f/pi_cH this call), message=''.
        On failure: converged=False, convergence_delta=inf, message explains
        why; spool-state fields reached before failure (tau_f, tau_cH, pi_f,
        pi_cH, tau_tL, pi_tL, M9, M19, alpha) are real, everything downstream
        of the nozzle exit state (thrust, fuel flow, mass flow, exit
        temperature/pressure/velocity) is NaN -- not computable once the core
        nozzle has collapsed, which is one of the two failure modes.

    Notes
    -----
    No compressor/turbine performance MAPS are required by this method:
    component efficiencies are held at their design (reference) values (or, if
    `fan_map`/`high_pressure_compressor_map` are given, drawn from those
    instead), and the operating point is found by scaling reference pressure/
    temperature ratios with choked-flow and power-balance relations (the "gas
    generator pumping characteristic"), generalized to the fan+LP-turbine /
    HP-compressor+HP-turbine spools.

    References
    ----------
    [1] Mattingly, J. D., "Elements of Gas Turbine Propulsion", 2nd ed., AIAA
        Education Series, 2005, Sec. 8.5 ("Turbofan Engine -- Separate
        Exhausts and Convergent Nozzles"), Eqs. (8.52a)-(8.52ak). Validated
        directly against the chapter's own worked numerical example
        (Example 8.8) to better than 0.1% on thrust, mass flow rate,
        fuel-air ratio, and specific fuel consumption.
    [2] Mattingly, J. D., Heiser, W. H., and Pratt, D. T., "Aircraft Engine
        Design", 2nd ed., AIAA Education Series, 2002, Sec. 5.3.5
        ("Component Matching") -- the constant-efficiency assumption's own
        stated validity range (accurate 70-100% of design corrected speed,
        degrading below 60%) and why it doesn't need engine speed in the
        off-design equations at all.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.solve_turbofan_offdesign_robust
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.design_turbofan_offdesign_matching
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
        tau_f, tau_tL, pi_tL = ref.tau_f, ref.tau_tL, ref.pi_tL
    else:
        tau_f, tau_tL, pi_tL = initial_guess

    # Shaft power offtake (IDG/motor), re-derived from the design point's specific work --
    # see the design_constants schema comment above for the full derivation. A no-offtake
    # engine has shaft_work_specific_design==0, making phi/phiR/P_offtake_design all zero
    # and reducing the tau_cH update below to exactly its original (pre-offtake) form.
    # getattr(...) keeps this backward compatible with a design_constants built without
    # this field (e.g. RESEARCH/22_ATI/Engine_Validation/validate_rcaide_offdesign_si.py's
    # standalone SI cross-check, which predates this addition).
    shaft_work_specific_design = getattr(dc, 'shaft_work_specific_design', 0.0)
    P_offtake_design = shaft_work_specific_design * ref.m0
    phiR = shaft_work_specific_design / (dc.cpt * ref.Tt4)

    alpha = ref.alpha
    mass_flow_rate_estimate = ref.m0
    converged = False
    collapsed = False
    for i in range(max_iterations):
        tau_tL_prev = tau_tL
        tau_f_prev = tau_f

        # HP compressor temperature ratio, from the HP-spool power balance. The X/XR ratio
        # (X = tau_lambda/(tau_r*tau_f)) is the original equation exactly; the phiR-phi term
        # is the offtake correction -- phi uses the *previous* iteration's mass flow estimate
        # (mass_flow_rate_estimate, updated below each pass) since this iteration's own mass
        # flow isn't known yet at this point (same Gauss-Seidel lagging the rest of this loop
        # already uses, e.g. tau_tL's pre-update pi_tL).
        X  = tau_lambda / (tau_r * tau_f)
        XR = tau_lambdaR / (tau_rR * ref.tau_f)
        shaft_work_specific = P_offtake_design / mass_flow_rate_estimate
        phi = shaft_work_specific / (dc.cpt * Tt4)
        tau_cH = 1 + (X / XR) * (ref.tau_cH - 1) + X * (phiR - phi)
        # HP compressor pressure ratio/efficiency -- map-consistent if high_pressure_compressor_map given,
        # else constant eta_cH
        pi_cH, eta_cH_used = compressor_pressure_ratio(tau_cH, dc.eta_cH, gamma_c, high_pressure_compressor_map)
        # likewise for the fan (combined with the LPC -- see LP-spool power balance below)
        pi_f, eta_f_used = compressor_pressure_ratio(tau_f, dc.eta_f, gamma_c, fan_map)

        # fan nozzle -- uses the fan ALONE, not fan+LPC combined: the bypass stream never
        # passes through the LPC (see the design_constants schema comment above for why
        # this split exists and was found necessary, not assumed)
        tau_f_alone = 1 + dc.fan_temperature_rise_fraction * (tau_f - 1)
        pi_f_alone = (1 + dc.eta_f_alone * (tau_f_alone - 1)) ** (gamma_c / (gamma_c - 1))
        Pt19_P0 = pi_r * pi_d * pi_f_alone * dc.pi_fn
        P19_P0, M19 = nozzle_state(Pt19_P0, gamma_c)

        # core nozzle
        Pt9_P0 = pi_r * pi_d * pi_f * pi_cH * dc.pi_b * dc.pi_tH * pi_tL * dc.pi_n
        P9_P0, M9 = nozzle_state(Pt9_P0, gamma_t)

        # bypass ratio -- floored, same rationale as compressor_pressure_ratio()'s floor above
        alpha = ref.alpha * (ref.pi_cH / pi_cH) * np.sqrt(max(
            (tau_lambda / (tau_r * tau_f)) / (tau_lambdaR / (tau_rR * ref.tau_f)), 1e-12
        )) * (mfp(M19, gamma_c, Rc) / mfp(ref.M19, gamma_c, Rc))

        # this iteration's own mass flow estimate (same relation used post-loop), carried
        # forward for the *next* pass's offtake term above -- lagged for the same reason
        # tau_cH above uses the previous pass's estimate, not this pass's. Tt4 floored, same
        # rationale as compressor_pressure_ratio()'s floor above: a mission-solver root-find
        # can probe a negative intermediate throttle (Tt4 = ref.Tt4*throttle), which used to
        # reach the M9==0 collapse check below before anything divided by it -- now this runs
        # first every pass, so it needs its own floor to keep failing cleanly downstream
        # instead of a raw ValueError/ZeroDivisionError on a doomed iterate.
        mass_flow_rate_estimate = ref.m0 * ((1 + alpha) / (1 + ref.alpha)) * \
            (P0 * pi_r * pi_d * pi_f * pi_cH) / (ref.P0 * pi_rR * pi_dR * ref.pi_f * ref.pi_cH) * \
            np.sqrt(ref.Tt4 / max(Tt4, 1e-6))

        # fan temperature ratio, from the LP-spool power balance -- under-relaxed
        tau_f_computed = 1 + ((1 - tau_tL) / (1 - ref.tau_tL)) * ((tau_lambda / tau_r) / (tau_lambdaR / tau_rR)) * \
            ((1 + ref.alpha) / (1 + alpha)) * (ref.tau_f - 1)
        tau_f = tau_f + relaxation_factor * (tau_f_computed - tau_f)

        # LP turbine temperature ratio from the current (pre-update) pi_tL
        tau_tL = 1 - dc.eta_tL * (1 - pi_tL ** ((gamma_t - 1) / gamma_t))

        # LP turbine pressure ratio, via choked-turbine-inlet flow matching -- under-relaxed.
        # M9=0 (collapsed core nozzle) would divide by zero below (mfp(0)=0); stop cleanly
        # instead -- solve_turbofan_offdesign_robust's continuation stepping is the real fix.
        if M9 == 0.0:
            collapsed = True
            break
        pi_tL_computed = ref.pi_tL * np.sqrt(tau_tL / ref.tau_tL) * (mfp(ref.M9, gamma_t, Rt) / mfp(M9, gamma_t, Rt))
        pi_tL = pi_tL + relaxation_factor * (pi_tL_computed - pi_tL)

        # i>0: pass 0 trivially reproduces reference tau_tL (update uses reference pi_tL
        # unchanged), so needs one real update behind it before comparing. Check tau_f too,
        # not just tau_tL: when the core nozzle is choked (M9=1 regardless of tau_f), the
        # pi_tL update ignores tau_f, so tau_tL alone can look converged from iteration 0
        # while tau_f is still actively converging.
        if i > 0 and abs(tau_tL - tau_tL_prev) < tolerance and abs(tau_f - tau_f_prev) < tolerance:
            converged = True
            break

    if collapsed:
        # Nothing downstream of the nozzle exit state is computable once Pt9/P0<1 (that's
        # the divide-by-zero this is dodging -- see the loop above) -- NaN for all of it,
        # not just skipped, so the schema is identical to the success case either way (a
        # caller can always read any field; only whether it's a real number differs).
        result = Data(
            tau_r=tau_r, pi_r=pi_r, pi_d=pi_d, tau_lambda=tau_lambda,
            tau_cH=tau_cH, pi_cH=pi_cH, tau_f=tau_f, pi_f=pi_f,
            tau_f_alone=tau_f_alone, pi_f_alone=pi_f_alone,
            tau_tL=tau_tL, pi_tL=pi_tL, alpha=alpha, M9=M9, M19=M19,
            stagnation_to_ambient_core_nozzle_pressure_ratio=Pt9_P0,
            stagnation_to_ambient_fan_nozzle_pressure_ratio=Pt19_P0,
            mass_flow_rate=np.nan, fuel_to_air_ratio=np.nan,
            thrust=np.nan, fuel_mass_flow_rate=np.nan, specific_fuel_consumption=np.nan,
            iterations=i + 1, converged=False, convergence_delta=np.inf,
            core_nozzle_exit_velocity=np.nan, fan_nozzle_exit_velocity=np.nan,
            core_nozzle_exit_static_temperature=np.nan, fan_nozzle_exit_static_temperature=np.nan,
            core_nozzle_exit_static_pressure=np.nan, fan_nozzle_exit_static_pressure=np.nan,
            core_nozzle_exit_stagnation_temperature=np.nan, fan_nozzle_exit_stagnation_temperature=np.nan,
            core_nozzle_exit_stagnation_pressure=np.nan, fan_nozzle_exit_stagnation_pressure=np.nan,
            eta_cH_used=eta_cH_used, eta_f_used=eta_f_used,
            message=f"core nozzle collapsed (Pt9/P0={Pt9_P0:.4f} < 1) at iteration {i}",
        )
    else:
        # Recompute final-state dependent quantities at whatever tau_tL/tau_f/pi_tL the
        # iteration reached -- the converged fixed point on success, or the last iterate if
        # max_iterations ran out without converging (M9!=0 either way, so none of this
        # divides by zero the way the collapsed branch above would).
        tau_cH = 1 + ((tau_lambda / tau_r) / (tau_lambdaR / tau_rR)) * (ref.tau_f / tau_f) * (ref.tau_cH - 1)
        pi_cH, eta_cH_used = compressor_pressure_ratio(tau_cH, dc.eta_cH, gamma_c, high_pressure_compressor_map)
        pi_f, eta_f_used = compressor_pressure_ratio(tau_f, dc.eta_f, gamma_c, fan_map)
        tau_f_alone = 1 + dc.fan_temperature_rise_fraction * (tau_f - 1)
        pi_f_alone = (1 + dc.eta_f_alone * (tau_f_alone - 1)) ** (gamma_c / (gamma_c - 1))
        Pt19_P0 = pi_r * pi_d * pi_f_alone * dc.pi_fn
        P19_P0, M19 = nozzle_state(Pt19_P0, gamma_c)
        Pt9_P0 = pi_r * pi_d * pi_f * pi_cH * dc.pi_b * dc.pi_tH * pi_tL * dc.pi_n
        P9_P0, M9 = nozzle_state(Pt9_P0, gamma_t)

        # engine mass flow rate -- uses the pressure ratio pi_r (not tau_r) at station 2: mass
        # flow through an area scales with Pt, i.e. P0*pi_r*pi_d, not the stagnation
        # temperature ratio. The temperature correction is the raw Tt4 ratio (station-4
        # choked-flow scaling): sqrt(Tt4R/Tt4), NOT sqrt(tau_lambdaR/tau_lambda), which would
        # double-count T0 (already present via P0/T0 in a0 and via pi_r) since
        # tau_lambda = cpt*Tt4/(cpc*T0).
        mass_flow_rate = ref.m0 * ((1 + alpha) / (1 + ref.alpha)) * \
            (P0 * pi_r * pi_d * pi_f * pi_cH) / (ref.P0 * pi_rR * pi_dR * ref.pi_f * ref.pi_cH) * \
            np.sqrt(ref.Tt4 / max(Tt4, 1e-6))

        # fuel/air ratio
        tau_x = tau_r * tau_f * tau_cH  # compressor-exit / T0 (station 3 temperature ratio)
        fuel_to_air_ratio = (tau_lambda - tau_x) / (dc.fuel_heating_value * dc.eta_b / (dc.cpc * T0) - tau_lambda)

        # core and fan nozzle exit temperature ratios. Need Pt/P at the exit plane (stagnation-
        # to-static), not P_exit/P0: Pt9_P0/P9_P0 gives that correctly in both the choked and
        # unchoked cases now that nozzle_state returns P/P0=1 when unchoked (P_exit=P0) --
        # ratio collapses to Pt9_P0 unchoked, pi_crit choked.
        Pt9_P9 = Pt9_P0 / P9_P0
        Pt19_P19 = Pt19_P0 / P19_P0
        T9_T0 = (tau_lambda * dc.tau_tH * tau_tL / (Pt9_P9 ** ((gamma_t - 1) / gamma_t))) * (dc.cpc / dc.cpt)
        T19_T0 = (tau_r * tau_f_alone) / (Pt19_P19 ** ((gamma_c - 1) / gamma_c))

        T9 = T9_T0 * T0
        P9 = P9_P0 * P0
        T19 = T19_T0 * T0
        P19 = P19_P0 * P0
        a9 = np.sqrt(gamma_t * Rt * T9)
        a19 = np.sqrt(gamma_c * Rc * T19)
        V9 = M9 * a9
        V19 = M19 * a19

        core_mass_flow_rate = mass_flow_rate / (1 + alpha)
        fan_mass_flow_rate = mass_flow_rate * alpha / (1 + alpha)
        core_thrust = core_mass_flow_rate * ((1 + fuel_to_air_ratio) * V9 - V0) + \
            (P9 - P0) * area_from_mass_flow_rate(core_mass_flow_rate * (1 + fuel_to_air_ratio), P9, T9, Rt, gamma_t, M9)
        fan_thrust = fan_mass_flow_rate * (V19 - V0) + \
            (P19 - P0) * area_from_mass_flow_rate(fan_mass_flow_rate, P19, T19, Rc, gamma_c, M19)
        thrust = core_thrust + fan_thrust

        fuel_mass_flow_rate = fuel_to_air_ratio * core_mass_flow_rate
        specific_fuel_consumption = fuel_mass_flow_rate / thrust  # kg/(N.s)

        # stagnation nozzle-exit conditions, for callers (e.g. compute_turbofan_performance_offdesign)
        # that want the full noise_conditions schema populated with real values rather than NaN
        Tt9 = T9 * (1 + (gamma_t - 1) / 2 * M9 ** 2)
        Pt9 = Pt9_P0 * P0
        Tt19 = T19 * (1 + (gamma_c - 1) / 2 * M19 ** 2)
        Pt19 = Pt19_P0 * P0

        convergence_delta = abs(tau_tL - tau_tL_prev)
        result = Data(
            tau_r=tau_r, pi_r=pi_r, pi_d=pi_d, tau_lambda=tau_lambda,
            tau_cH=tau_cH, pi_cH=pi_cH, tau_f=tau_f, pi_f=pi_f,
            tau_f_alone=tau_f_alone, pi_f_alone=pi_f_alone,
            tau_tL=tau_tL, pi_tL=pi_tL, alpha=alpha, M9=M9, M19=M19,
            stagnation_to_ambient_core_nozzle_pressure_ratio=Pt9_P0,
            stagnation_to_ambient_fan_nozzle_pressure_ratio=Pt19_P0,
            mass_flow_rate=mass_flow_rate, fuel_to_air_ratio=fuel_to_air_ratio,
            thrust=thrust, fuel_mass_flow_rate=fuel_mass_flow_rate,
            specific_fuel_consumption=specific_fuel_consumption,
            iterations=i + 1, converged=converged, convergence_delta=convergence_delta,
            core_nozzle_exit_velocity=V9, fan_nozzle_exit_velocity=V19,
            core_nozzle_exit_static_temperature=T9, fan_nozzle_exit_static_temperature=T19,
            core_nozzle_exit_static_pressure=P9, fan_nozzle_exit_static_pressure=P19,
            core_nozzle_exit_stagnation_temperature=Tt9, fan_nozzle_exit_stagnation_temperature=Tt19,
            core_nozzle_exit_stagnation_pressure=Pt9, fan_nozzle_exit_stagnation_pressure=Pt19,
            eta_cH_used=eta_cH_used, eta_f_used=eta_f_used,
            message='' if converged else f"did not converge in {max_iterations} iterations "
                                          f"(last delta={convergence_delta:.2e})",
        )

    return result


# ----------------------------------------------------------------------------------------------------------------------
#  solve_turbofan_offdesign_robust
# ----------------------------------------------------------------------------------------------------------------------
def solve_turbofan_offdesign_robust(design_constants, reference_point, mach_number, static_temperature,
                                     static_pressure, combustor_exit_temperature, tolerance=1e-8, max_iterations=200,
                                     relaxation_factor=0.5, max_continuation_steps=32, fan_map=None,
                                     high_pressure_compressor_map=None, allow_unconverged_fallback=False):
    """
    Robust wrapper around `solve_turbofan_offdesign`: tries a direct solve
    first, falls back to *continuation* if that fails (stepping from the
    reference condition to the target in increasing increments, up to
    `max_continuation_steps`, chaining each step's converged state as the
    next step's guess -- fixes bad-first-pass failures that under-relaxation
    can't, since relaxation only damps updates and this can fail before any
    update is applied), and retries continuation once more with
    `fan_map`/`high_pressure_compressor_map` active if given. Maps are a
    third tier, not folded into the first two, so a point that already
    converges without one keeps the exact constant-efficiency answer rather
    than a map-based approximation -- and per `Generic_Compressor_Map`'s own
    validation, a map is not a complete part-power fix by itself (it changes
    how the pressure ratio splits for a given work demand, not the demand
    itself), so it's a fallback, not a guarantee of convergence.

    Parameters
    ----------
    allow_unconverged_fallback : bool, optional
        If every tier fails, raises `OffDesignMatchingError` by default. If
        `True`, returns the best (smallest convergence delta) partial result
        seen, with `converged=False`. Deliberately not the default: RCAIDE's
        existing fixed-pressure-ratio analytical model silently clips an
        invalid state to a plausible-looking but wrong number in the same
        situation -- this module fails loudly by default to not repeat that.

    Returns
    -------
    Data
        Same fields as `solve_turbofan_offdesign`.

    Raises
    ------
    OffDesignMatchingError
        If every tier fails and `allow_unconverged_fallback` is False (the
        default).
    """
    best_partial = None

    def track_best(result):
        # collapsed-nozzle results carry convergence_delta=inf specifically so they're
        # skipped here -- there's nothing usable to fall back to from one (see
        # solve_turbofan_offdesign's Returns docstring), unlike a genuine did-not-converge-
        # in-max_iterations result, which has a real (finite) delta
        nonlocal best_partial
        if not np.isfinite(result.convergence_delta):
            return
        if best_partial is None or result.convergence_delta < best_partial.convergence_delta:
            best_partial = result

    def continuation(fan_map_used, hpc_map_used):
        n_steps = 2
        while n_steps <= max_continuation_steps:
            state = (reference_point.tau_f, reference_point.tau_tL, reference_point.pi_tL)
            result = None
            for k in range(1, n_steps + 1):
                frac = k / n_steps
                M0_k = reference_point.M0 + frac * (mach_number - reference_point.M0)
                T0_k = reference_point.T0 + frac * (static_temperature - reference_point.T0)
                P0_k = reference_point.P0 + frac * (static_pressure - reference_point.P0)
                Tt4_k = reference_point.Tt4 + frac * (combustor_exit_temperature - reference_point.Tt4)
                result = solve_turbofan_offdesign(design_constants, reference_point, M0_k, T0_k, P0_k, Tt4_k,
                                                   tolerance=tolerance, max_iterations=max_iterations,
                                                   relaxation_factor=relaxation_factor, initial_guess=state,
                                                   fan_map=fan_map_used, high_pressure_compressor_map=hpc_map_used)
                if not result.converged:
                    break
                state = (result.tau_f, result.tau_tL, result.pi_tL)
            if result.converged:
                return result
            track_best(result)
            n_steps *= 2
        return None

    direct = solve_turbofan_offdesign(design_constants, reference_point, mach_number, static_temperature,
                                       static_pressure, combustor_exit_temperature, tolerance=tolerance,
                                       max_iterations=max_iterations, relaxation_factor=relaxation_factor)
    result = direct if direct.converged else None
    if result is None:
        track_best(direct)
        result = continuation(None, None)

    if result is None and (fan_map is not None or high_pressure_compressor_map is not None):
        result = continuation(fan_map, high_pressure_compressor_map)

    if result is None and allow_unconverged_fallback:
        result = best_partial

    if result is None:
        raise OffDesignMatchingError(
            f"off-design matching failed even with {max_continuation_steps}-step continuation "
            f"(map-based retry {'attempted' if (fan_map is not None or high_pressure_compressor_map is not None) else 'not available'}) "
            f"from the reference point (target M0={mach_number}, T0={static_temperature}, "
            f"P0={static_pressure}, Tt4={combustor_exit_temperature})"
        )
    return result
