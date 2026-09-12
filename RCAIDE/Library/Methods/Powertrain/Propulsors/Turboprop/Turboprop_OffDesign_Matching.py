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
    mfp, compressor_pressure_ratio, nozzle_state, OffDesignMatchingError)

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  design_constants / reference_point schema -- see design_turboprop_offdesign_matching
# ----------------------------------------------------------------------------------------------------------------------
# design_constants: gamma_c/gamma_t, cpc/cpt, fuel_heating_value, pi_dmax, pi_b, pi_n,
# tau_tH/pi_tH (gas-generator turbine, held constant), eta_c, eta_b, eta_mH, eta_mL, eta_tL,
# eta_prop, eta_gearbox, shaft_work_specific_design (gas-generator accessory offtake,
# 0 if none) -- all SI units.
# reference_point: M0, T0, P0, Tt4, pi_c/tau_c, tau_tL/pi_tL, M9, m0, F (design-point state;
# every off-design solve scales relative to this).

# ----------------------------------------------------------------------------------------------------------------------
#  solve_turboprop_offdesign
# ----------------------------------------------------------------------------------------------------------------------
def solve_turboprop_offdesign(design_constants, reference_point, mach_number, static_temperature, static_pressure,
                               combustor_exit_temperature, tolerance=1e-8, max_iterations=200, relaxation_factor=0.5,
                               initial_guess=None):
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
    turbine. The gas-generator spool (compressor + HP turbine) matches
    `solve_turbojet_offdesign`'s single compressor-turbine pairing exactly.

    Thrust reuses RCAIDE's own validated work-interaction-coefficient
    formula from `Turboprop/compute_thrust.py` directly (`Ccore + Cprop`)
    rather than a re-derived momentum equation -- cross-checked against
    Cantwell Ref. [2] Ch. 6 Eqs. (6.22)-(6.27) and Mattingly/Heiser/Pratt
    Ref. [1] Appendix K Eqs. (K.6)-(K.8), which agree with it term-for-term.

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
    tolerance, max_iterations, relaxation_factor : optional
        Same rationale as `solve_turbofan_offdesign`'s own (see its
        docstring) -- limit-cycling at extreme off-design points, not wrong
        equations.
    initial_guess : tuple of float, optional
        (tau_c, tau_tL, pi_tL) starting guess, used instead of the reference
        point's own values -- for `solve_turboprop_offdesign_robust`'s
        continuation stepping.

    Returns
    -------
    Data
        Never raises -- check `.converged` instead. On success: tau_c, pi_c,
        tau_tL, pi_tL, M9, mass_flow_rate [kg/s], fuel_to_air_ratio, thrust
        [N], fuel_mass_flow_rate [kg/s], specific_fuel_consumption
        [kg/(N.s)], power [W] (`Cprop` converted to power, i.e. the
        propeller's net *thrust-equivalent* work rate -- already carrying
        the `eta_prop*eta_gearbox*eta_mL` chain, so it reads well below the
        raw specific work the free turbine actually extracts from the gas
        (`turboprop.design_power` at the reference point); NOT the same
        quantity as `compute_thrust.py`'s own `turboprop_conditions.power`,
        which is propulsive power, thrust times flight velocity),
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
    dc, ref = design_constants, reference_point
    # Floored, not the raw mach_number: Fsp below divides by V0=a0*M0, singular at M0=0 --
    # same static-point convention design_turboprop.py's own SLS step already uses (M0=0.01,
    # not exactly 0), so a deck's M0=0 row (needed for Turbofan_Surrogate's SLS reference row)
    # comes out finite and consistent with that convention instead of raising ZeroDivisionError.
    M0, T0, P0, Tt4 = max(mach_number, 0.01), static_temperature, static_pressure, combustor_exit_temperature

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

    tau_rR = 1 + (gamma_c - 1) / 2 * ref.M0 ** 2
    pi_rR = tau_rR ** (gamma_c / (gamma_c - 1))
    eta_rR = 1.0 if ref.M0 <= 1 else 1 - 0.075 * (ref.M0 - 1) ** 1.35
    pi_dR = dc.pi_dmax * eta_rR
    tau_lambdaR = dc.cpt * ref.Tt4 / (dc.cpc * ref.T0)

    if initial_guess is None:
        tau_c, tau_tL, pi_tL = ref.tau_c, ref.tau_tL, ref.pi_tL
    else:
        tau_c, tau_tL, pi_tL = initial_guess

    # Gas-generator accessory offtake (IDG/motor), same derivation as
    # solve_turbojet_offdesign's tau_cH extension -- see its schema comment. Zero for
    # an engine with no offtake, reducing this exactly to the original equation.
    shaft_work_specific_design = getattr(dc, 'shaft_work_specific_design', 0.0)
    P_offtake_design = shaft_work_specific_design * ref.m0
    phiR = shaft_work_specific_design / (dc.cpt * ref.Tt4)

    mass_flow_rate_estimate = ref.m0
    converged = False
    collapsed = False
    for i in range(max_iterations):
        tau_tL_prev = tau_tL
        tau_c_prev = tau_c

        # gas-generator spool power balance -- same form as solve_turbojet_offdesign's
        # tau_cH, renamed (the compressor here IS the whole gas-generator spool)
        tau_cH_ratio = (tau_lambda / tau_r) / (tau_lambdaR / tau_rR)
        shaft_work_specific = P_offtake_design / mass_flow_rate_estimate
        phi = shaft_work_specific / (dc.cpt * Tt4)
        tau_c_new = 1 + tau_cH_ratio * (ref.tau_c / tau_c) * (ref.tau_c - 1) + \
            (tau_lambda / tau_r) * (phiR - phi)
        pi_c, eta_c_used = compressor_pressure_ratio(tau_c_new, dc.eta_c, gamma_c)
        tau_c = tau_c_new

        # core nozzle -- convergent (Turboprop uses Expansion_Nozzle, not Turbojet's
        # Supersonic_Nozzle), same nozzle_state as the turbofan solver
        Pt9_P0 = pi_r * pi_d * pi_c * dc.pi_b * dc.pi_tH * pi_tL * dc.pi_n
        P9_P0, M9 = nozzle_state(Pt9_P0, gamma_t)

        if M9 == 0.0:
            collapsed = True
            break

        # carried forward for next pass's offtake term above, lagged like pi_tL below
        mass_flow_rate_estimate = ref.m0 * (P0 * pi_r * pi_d * pi_c) / (ref.P0 * pi_rR * pi_dR * ref.pi_c) * \
            np.sqrt(ref.Tt4 / max(Tt4, 1e-6))

        # free turbine pressure ratio -- mass-flow matched to the core nozzle, same
        # equation as solve_turbofan_offdesign's LP turbine, no compressor on this shaft
        pi_tL_computed = ref.pi_tL * np.sqrt(tau_tL / ref.tau_tL) * (mfp(ref.M9, gamma_t, Rt) / mfp(M9, gamma_t, Rt))
        pi_tL = pi_tL + relaxation_factor * (pi_tL_computed - pi_tL)
        tau_tL = 1 - dc.eta_tL * (1 - pi_tL ** ((gamma_t - 1) / gamma_t))

        if i > 0 and abs(tau_tL - tau_tL_prev) < tolerance and abs(tau_c - tau_c_prev) < tolerance:
            converged = True
            break

    if collapsed:
        result = Data(
            tau_r=tau_r, pi_r=pi_r, pi_d=pi_d, tau_lambda=tau_lambda,
            tau_c=tau_c, pi_c=pi_c, tau_tL=tau_tL, pi_tL=pi_tL, M9=np.nan,
            stagnation_to_ambient_core_nozzle_pressure_ratio=Pt9_P0,
            mass_flow_rate=np.nan, fuel_to_air_ratio=np.nan,
            thrust=np.nan, fuel_mass_flow_rate=np.nan, specific_fuel_consumption=np.nan,
            power=np.nan, iterations=i + 1, converged=False, convergence_delta=np.inf,
            core_nozzle_exit_velocity=np.nan,
            core_nozzle_exit_static_temperature=np.nan, core_nozzle_exit_static_pressure=np.nan,
            core_nozzle_exit_stagnation_temperature=np.nan, core_nozzle_exit_stagnation_pressure=np.nan,
            eta_c_used=eta_c_used,
            message=f"core nozzle collapsed (Pt9/P0={Pt9_P0:.4f} < 1) at iteration {i}",
        )
    else:
        tau_c_new = 1 + ((tau_lambda / tau_r) / (tau_lambdaR / tau_rR)) * (ref.tau_c / tau_c) * (ref.tau_c - 1)
        pi_c, eta_c_used = compressor_pressure_ratio(tau_c_new, dc.eta_c, gamma_c)
        tau_c = tau_c_new
        Pt9_P0 = pi_r * pi_d * pi_c * dc.pi_b * dc.pi_tH * pi_tL * dc.pi_n
        P9_P0, M9 = nozzle_state(Pt9_P0, gamma_t)

        mass_flow_rate = ref.m0 * (P0 * pi_r * pi_d * pi_c) / (ref.P0 * pi_rR * pi_dR * ref.pi_c) * \
            np.sqrt(ref.Tt4 / Tt4)

        tau_x = tau_r * tau_c  # compressor-exit / T0 (station 3 temperature ratio)
        fuel_to_air_ratio = (tau_lambda - tau_x) / (dc.fuel_heating_value * dc.eta_b / (dc.cpc * T0) - tau_lambda)

        Pt9_P9 = Pt9_P0 / P9_P0
        T9_T0 = (tau_lambda * dc.tau_tH * tau_tL / (Pt9_P9 ** ((gamma_t - 1) / gamma_t))) * (dc.cpc / dc.cpt)
        T9 = T9_T0 * T0
        P9 = P9_P0 * P0
        a9 = np.sqrt(gamma_t * Rt * T9)
        V9 = M9 * a9

        # thrust -- RCAIDE's own work-interaction-coefficient formula, see docstring
        f = fuel_to_air_ratio
        Ccore = (gamma_c - 1) * M0 * (
            (1 + f) * (V9 / a0) - M0 +
            (1 + f) * (Rt / Rc) * ((T9 / T0) / (V9 / a0)) * ((1 - P0 / P9) / gamma_c)
        )
        Cprop = dc.eta_prop * dc.eta_gearbox * dc.eta_mL * (1 + f) * (dc.cpt * Tt4) / (dc.cpc * T0) * \
            dc.tau_tH * (1 - tau_tL)
        Ctotal = Ccore + Cprop

        Fsp = Ctotal * dc.cpc * T0 / V0
        thrust = Fsp * mass_flow_rate

        fuel_mass_flow_rate = fuel_to_air_ratio * mass_flow_rate
        specific_fuel_consumption = fuel_mass_flow_rate / thrust  # kg/(N.s)
        power = Cprop * dc.cpc * T0 * mass_flow_rate  # shaft power to the propeller [W]

        Tt9 = T9 * (1 + (gamma_t - 1) / 2 * M9 ** 2)
        Pt9 = Pt9_P0 * P0

        convergence_delta = abs(tau_tL - tau_tL_prev)
        result = Data(
            tau_r=tau_r, pi_r=pi_r, pi_d=pi_d, tau_lambda=tau_lambda,
            tau_c=tau_c, pi_c=pi_c, tau_tL=tau_tL, pi_tL=pi_tL, M9=M9,
            stagnation_to_ambient_core_nozzle_pressure_ratio=Pt9_P0,
            mass_flow_rate=mass_flow_rate, fuel_to_air_ratio=fuel_to_air_ratio,
            thrust=thrust, fuel_mass_flow_rate=fuel_mass_flow_rate,
            specific_fuel_consumption=specific_fuel_consumption, power=power,
            iterations=i + 1, converged=converged, convergence_delta=convergence_delta,
            core_nozzle_exit_velocity=V9,
            core_nozzle_exit_static_temperature=T9, core_nozzle_exit_static_pressure=P9,
            core_nozzle_exit_stagnation_temperature=Tt9, core_nozzle_exit_stagnation_pressure=Pt9,
            eta_c_used=eta_c_used,
            message='' if converged else f"did not converge in {max_iterations} iterations "
                                          f"(last delta={convergence_delta:.2e})",
        )

    return result


# ----------------------------------------------------------------------------------------------------------------------
#  solve_turboprop_offdesign_robust
# ----------------------------------------------------------------------------------------------------------------------
def solve_turboprop_offdesign_robust(design_constants, reference_point, mach_number, static_temperature,
                                      static_pressure, combustor_exit_temperature, tolerance=1e-8, max_iterations=200,
                                      relaxation_factor=0.5, max_continuation_steps=32, allow_unconverged_fallback=False):
    """
    Robust wrapper around `solve_turboprop_offdesign`: tries a direct solve
    first, falls back to continuation if that fails. Same rationale and
    behavior as `Turbofan_OffDesign_Matching.solve_turbofan_offdesign_robust`
    (see its docstring) -- no map-based fallback tier here.

    Returns
    -------
    Data
        Same fields as `solve_turboprop_offdesign`.

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
                result = solve_turboprop_offdesign(design_constants, reference_point, M0_k, T0_k, P0_k, Tt4_k,
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

    direct = solve_turboprop_offdesign(design_constants, reference_point, mach_number, static_temperature,
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
            f"turboprop off-design matching failed even with {max_continuation_steps}-step continuation "
            f"from the reference point (target M0={mach_number}, T0={static_temperature}, "
            f"P0={static_pressure}, Tt4={combustor_exit_temperature})"
        )
    return result
