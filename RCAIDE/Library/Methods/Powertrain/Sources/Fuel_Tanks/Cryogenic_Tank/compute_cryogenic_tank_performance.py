# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/Cryogenic_Tank/compute_cryogenic_tank_performance.py
#
# Created: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
from scipy.optimize import brentq
from scipy.integrate import solve_ivp

import RCAIDE
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_tank_heat_leak import compute_cryogenic_tank_heat_leak, compute_cryogenic_tank_heat_leak_cuboid
from RCAIDE.Framework.Core.Physical_Constants import UNIVERSAL_GAS_CONSTANT
from RCAIDE.Framework.Core import Units

# Skips re-solving when a fsolve probe couldn't have changed this tank's inputs.
def _cache_key_matches(cached, current):
    if cached is None or len(cached) != len(current):
        return False
    # equal_nan=True: refuel_target_mass is NaN outside Refuel segments, and NaN != NaN
    # under the default would defeat the cache (force a full re-solve) on every call.
    return all(np.array_equal(a, b, equal_nan=True) for a, b in zip(cached, current))

# ----------------------------------------------------------------------------------------------------------------------
#  Cryogenic Tank In-Flight Boil-Off Performance
# ----------------------------------------------------------------------------------------------------------------------
def compute_cryogenic_tank_performance(tank, state, network, rtol=1e-4, atol=1e-7, method='LSODA'):
    """
    Two-phase lumped-parameter in-flight boil-off model for a cryogenic fuel tank.

    Solved as its own internal, decoupled initial-value-problem integration (adaptive
    stiff-ODE Radau via scipy), NOT as mission-level Newton unknowns/residuals -- see
    ``append_cryogenic_tank_unknown_and_residual.py`` for the earlier coupled-unknown
    version and why it was abandoned. Each call integrates the tank's 4-state ODE
    system (ullage/liquid mass, ullage/liquid temperature) forward from its known
    node-0 initial condition across the segment's full time span, using the mission's own
    driving-input trajectory (engine fuel demand, freestream conditions) interpolated
    between the mission's (sparse, e.g. 16-point Chebyshev) collocation nodes so the
    adaptive integrator can resolve dynamics between them, then samples the result
    back at exactly those collocation points via ``t_eval``. This is the same
    warm-start Radau approach already verified in isolation
    (``VnV/Verification/powertrain/cryogenic_tank_performance_test.py``,
    ``RESEARCH/.../cryogenic_tank_boil_off_validation.py``), just run for real inside
    the network's own iterate instead of as a standalone script. Decoupling it from
    the mission solver removes the 24 extra coupled unknowns (6 states x 4 tanks) that
    were preventing the full aircraft-level mission from converging as mission-level
    unknowns, at the cost of a fresh internal IVP solve on every outer network iterate
    rather than a single shared Newton solve across the whole aircraft.

    Fluid properties come from the saturated-property tables already bundled with the
    propellant classes (``tank.fuel.cryogen_properties`` /
    ``tank.fuel.saturation_temperature``); the ullage is treated as an ideal gas off
    the saturation dome.

    The instantaneous boil-off/vent split uses a smoothed complementarity relaxation
    (rather than a hard threshold) so the ODE right-hand side stays differentiable
    (the implicit integrators below need a well-behaved Jacobian): when more boil-off
    is needed than passive heat leak alone provides, the excess is treated as
    active-heater duty (and its required power reported as an output, not fed back
    into the aircraft's electrical bus); when less is needed, the excess ullage vapor
    is vented.

    ``method``/``rtol``/``atol`` default to LSODA/1e-4/1e-7 -- chosen from a direct
    speed/accuracy benchmark against a tight-tolerance (1e-9/1e-12) Radau truth
    trajectory: LSODA at this tolerance is ~3x faster per solve than the originally
    used Radau/1e-6/1e-9 while still holding a ~10x accuracy margin under the 1e-2
    relative-error threshold the VnV/RESEARCH validation scripts check against
    (BDF was similarly fast but had far less margin, ~9.7e-3 at the same tolerance --
    right at that threshold -- so it was not used). This matters because every call
    here is now a full internal solve run fresh on every outer network iterate,
    including fsolve's own finite-difference Jacobian probes of the *other* (now
    tank-unrelated) mission unknowns -- there is no way from inside this function to
    tell a probe that could not have changed the tank's own inputs from one that
    could, so solver speed here directly multiplies the full mission's wall-clock
    time. ``rtol``/``atol``/``method`` are all exposed as kwargs so a caller can
    re-solve the same problem at a tighter tolerance/different method as an
    independent cross-check, the way the VnV/RESEARCH validation scripts already
    verify their own IVP truth trajectory against the production solve.

    Notes
    -----
    Not modeled here (out of scope for this port): active-heater dynamics (treated as
    quasi-steady, no lag), bulk boiling / cloud condensation corrections, and pump
    power for active re-liquefaction. Refueling (``refuel_mass_flow_rate``) assumes
    incoming fuel enters already at the tank's own bulk liquid temperature -- no
    chill-down transient.

    When ``tank_conditions.refuel_target_mass`` is set (a Ground.Refuel segment),
    ``refuel_mass_flow_rate`` isn't just applied blindly for the whole segment: a
    terminal IVP event detects fuel_mass crossing that target and the fill is cut to
    zero from that point on, mirroring Battery_Recharge's cutoff_SOC. Without this,
    a constant rate sized to reach the target by segment end (ignoring the boil-off
    that keeps draining liquid mass throughout the same window, since ground ops
    still have ambient heat leak) systematically undershoots -- the tank never
    actually reads full. A second terminal event watches V_l against
    tank.volume_properties.net_volume and can cut the fill off first (flagged in
    ``refuel_volume_capped``): refuel_target_mass is a fixed mass computed at
    design temperature, but a tank that has warmed during the day holds more
    volume per kg of liquid, so the mass target alone can let V_l run past
    net_volume and collapse the ullage.
    """
    tag             = tank.tag
    tank_conditions = state.conditions.energy.sources[tag]
    fuel            = tank.fuel

    t = np.ravel(state.numerics.time.control_points)

    (T_l_lo, T_l_hi), (P_lo, P_hi) = fuel.property_table_range(phase='liquid')
    (T_g_lo, T_g_hi), _            = fuel.property_table_range(phase='vapor')
    R_specific = UNIVERSAL_GAS_CONSTANT / fuel.molecular_weight  # J/(kg*K)

    # ------------------------------------------------------------------
    #  Driving-input trajectories at the mission's own (sparse) collocation
    #  points -- np.interp below resamples these onto whatever intermediate
    #  times Radau's adaptive stepper actually queries.
    # ------------------------------------------------------------------
    chemical_power_pts = tank_conditions.power_split_ratio[:,0] * \
        state.conditions.energy.distributors[tank.assigned_distributors[0][0]].outputs.power.chemical[:,0]
    T_env_pts   = state.conditions.freestream.temperature[:,0]
    nu_air_pts  = state.conditions.freestream.kinematic_viscosity[:,0]
    Pr_air_pts  = state.conditions.freestream.prandtl_number[:,0]
    k_air_pts   = state.conditions.freestream.thermal_conductivity[:,0]
    refuel_pts  = tank_conditions.refuel_mass_flow_rate[:,0]

    y0 = np.array([
        tank_conditions.ullage_mass[0,0],
        tank_conditions.fuel_mass[0,0],
        tank_conditions.ullage_temperature[0,0],
        tank_conditions.fuel_temperature[0,0],
    ])

    refuel_target_arr = tank_conditions.refuel_target_mass[:,0]
    cache_key = (t, chemical_power_pts, T_env_pts, nu_air_pts, Pr_air_pts, k_air_pts, refuel_pts, refuel_target_arr, y0)
    cached    = getattr(tank_conditions, '_boil_off_solve_cache', None)
    if cached is not None and _cache_key_matches(cached[0], cache_key):
        return cached[1]

    max_heater_power = 10.0 * tank.design_total_heat_transfer
    if network is not None:
        for converter in getattr(network, 'converters', []):
            if isinstance(converter, RCAIDE.Library.Components.Powertrain.Converters.Heater) and \
               getattr(converter, 'assigned_tank', None) == tag and converter.rated_power is not None:
                max_heater_power = converter.rated_power
                break

    if t.size < 2:
        m_g0, m_l0, T_g0, T_l0 = (np.array([v]) for v in y0)

        _, diag = _tank_state_rates(
            tank, fuel, R_specific, m_g0, m_l0, T_g0, T_l0,
            chemical_power_pts, T_env_pts, nu_air_pts, Pr_air_pts, k_air_pts, refuel_pts,
            T_g_lo, T_g_hi, T_l_lo, T_l_hi, P_lo, P_hi, max_heater_power)

        tank_conditions.ullage_volume[:,0]      = diag['V_g']
        tank_conditions.fuel_volume[:,0]        = diag['V_l']
        tank_conditions.pressure[:,0]           = diag['P']
        tank_conditions.vent_rate[:,0]          = diag['m_dot_vent']
        tank_conditions.boil_off_flow_rate[:,0] = diag['m_dot_bo_final']
        tank_conditions.heater_power[:,0]       = diag['heater_power']
        tank_conditions.mass_flow_rate[:,0]         = diag['m_dot_l_engine'] + diag['m_dot_vent'] - refuel_pts
        tank_conditions.outputs.power.chemical[:,0] = diag['m_dot_l_engine'] * fuel.lower_heating_value
        tank_conditions.environmental_heat_leak_liquid[:,0] = diag['Q_e_l']
        tank_conditions.environmental_heat_leak_ullage[:,0] = diag['Q_e_g']
        tank_conditions.temperature_out_of_range[:,0]  = diag['T_out_of_range']
        tank_conditions.liquid_thermal_floor_active[:,0] = diag['m_l_thermal_floored']
        tank_conditions.liquid_mass_floor_blend[:,0]   = diag['m_barrier_blend']
        tank_conditions.liquid_availability_gate[:,0]  = diag['m_dot_avail']
        tank_conditions.heater_saturated[:,0]          = diag['heater_saturated']
        tank_conditions.refuel_volume_capped[:,0]      = 0.0

        result = tank_conditions.inputs, tank_conditions.outputs, True, tank.tag
        tank_conditions._boil_off_solve_cache = (tuple(np.array(k, copy=True) for k in cache_key), result)
        return result

    duration = t[-1] - t[0]

    # Scale atol per-state instead of one scalar across mass/temp.
    state_floors = np.array([1e-3, 1e-3, 1.0, 1.0])
    atol_vec = atol * np.maximum(np.abs(y0), state_floors)

    def _build_rhs(refuel_pts_local):
        def rhs(t_i, y):
            m_g, m_l, T_g, T_l = y
            chemical_power = np.interp(t_i, t, chemical_power_pts)
            T_env          = np.interp(t_i, t, T_env_pts)
            nu_air         = np.interp(t_i, t, nu_air_pts)
            Pr_air         = np.interp(t_i, t, Pr_air_pts)
            k_air          = np.interp(t_i, t, k_air_pts)
            refuel         = np.interp(t_i, t, refuel_pts_local)
            (dm_g, dm_l, dT_g, dT_l), _ = _tank_state_rates(
                tank, fuel, R_specific,
                np.array([m_g]), np.array([m_l]), np.array([T_g]), np.array([T_l]),
                np.array([chemical_power]), np.array([T_env]), np.array([nu_air]), np.array([Pr_air]), np.array([k_air]), np.array([refuel]),
                T_g_lo, T_g_hi, T_l_lo, T_l_hi, P_lo, P_hi, max_heater_power)
            return [dm_g[0], dm_l[0], dT_g[0], dT_l[0]]
        return rhs

    refuel_target = tank_conditions.refuel_target_mass[0, 0]
    net_volume    = tank.volume_properties.net_volume
    filling       = np.isfinite(refuel_target) and y0[1] < refuel_target
    volume_capped = False

    if filling:
        # Terminal event 1: stop the "filling" phase the instant fuel_mass reaches
        # the target, the same cutoff Battery_Recharge applies at cutoff_SOC.
        # direction=1 requires an upward crossing, so a tank already above target
        # (handled by the `filling` guard above) can't spuriously retrigger it.
        def _tank_full(t_i, y):
            return y[1] - refuel_target
        _tank_full.terminal  = True
        _tank_full.direction = 1

        # Terminal event 2: stop early if V_l reaches net_volume regardless of
        # mass. refuel_target_mass is a fixed density-at-design-temperature
        # figure, but the tank's real physical limit is volume -- V_l grows
        # faster than the mass target alone implies whenever T_l has warmed
        # above the design temperature (liquid is less dense), which is
        # exactly the case for a tank that has been draining/warming all day
        # before an end-of-day refuel. Without this, the mass-only cutoff can
        # let V_l run past net_volume, collapsing the ullage and blowing up
        # P = m_g*R*T_g/V_g as V_g -> 0.
        def _tank_volume_full(t_i, y):
            m_l, T_l = y[1], y[3]
            T_l_c = np.clip(T_l, T_l_lo, T_l_hi)
            rho_l = fuel.cryogen_properties(np.array([T_l_c]), "Density (kg/m3)", phase='liquid')[0]
            return m_l / rho_l - net_volume
        _tank_volume_full.terminal  = True
        _tank_volume_full.direction = 1

        rhs_fill = _build_rhs(refuel_pts)
        sol      = solve_ivp(rhs_fill, (t[0], t[-1]), y0, method=method, t_eval=t,
                              rtol=rtol, atol=atol_vec, events=[_tank_full, _tank_volume_full])
        if not sol.success:
            raise RuntimeError(
                f"Cryogenic tank '{tag}' internal boil-off IVP failed to integrate: {sol.message}")

        refuel_pts_eff = refuel_pts
        fired_events = [i for i, te in enumerate(sol.t_events) if te.size]
        if fired_events:  # reached target or volume limit before segment end -> splice in a zero-fill hold
            event_idx        = fired_events[0]
            volume_capped    = (event_idx == 1)
            t_event, y_event = sol.t_events[event_idx][0], sol.y_events[event_idx][0]

            t_before = t[t <= t_event]
            t_after  = t[t >  t_event]

            sol_before = solve_ivp(rhs_fill, (t[0], t_event), y0, method=method,
                                    t_eval=t_before, rtol=rtol, atol=atol_vec)
            if not sol_before.success:
                raise RuntimeError(
                    f"Cryogenic tank '{tag}' internal boil-off IVP failed to integrate (fill phase): {sol_before.message}")

            if t_after.size:
                rhs_hold   = _build_rhs(np.zeros_like(refuel_pts))
                sol_after  = solve_ivp(rhs_hold, (t_event, t[-1]), y_event, method=method,
                                        t_eval=t_after, rtol=rtol, atol=atol_vec)
                if not sol_after.success:
                    raise RuntimeError(
                        f"Cryogenic tank '{tag}' internal boil-off IVP failed to integrate (post-full hold): {sol_after.message}")
                y_all = np.concatenate([sol_before.y, sol_after.y], axis=1)
            else:
                y_all = sol_before.y

            m_g, m_l, T_g, T_l = y_all
            refuel_pts_eff = np.where(t <= t_event, refuel_pts, 0.0)
        else:
            m_g, m_l, T_g, T_l = sol.y
    else:
        if np.isfinite(refuel_target):
            # Already at/above target (e.g. topping off a nearly-full tank) -- don't add fuel.
            refuel_pts = np.zeros_like(refuel_pts)
        rhs = _build_rhs(refuel_pts)
        sol = solve_ivp(rhs, (t[0], t[-1]), y0, method=method, t_eval=t, rtol=rtol, atol=atol_vec)
        if not sol.success:
            raise RuntimeError(
                f"Cryogenic tank '{tag}' internal boil-off IVP failed to integrate: {sol.message}")
        m_g, m_l, T_g, T_l = sol.y
        refuel_pts_eff = refuel_pts

    # One extra vectorized call (cheap -- same array-length-n_nodes shape the
    # old residual-based version always used) to recover the diagnostic
    # outputs (pressure, vent/boil-off split, heater power) at exactly the
    # collocation points, using their own real (not re-interpolated) inputs.
    _, diag = _tank_state_rates(
        tank, fuel, R_specific, m_g, m_l, T_g, T_l,
        chemical_power_pts, T_env_pts, nu_air_pts, Pr_air_pts, k_air_pts, refuel_pts_eff,
        T_g_lo, T_g_hi, T_l_lo, T_l_hi, P_lo, P_hi, max_heater_power)

    # ------------------------------------------------------------------
    #  Store outputs
    # ------------------------------------------------------------------
    tank_conditions.ullage_mass[:,0]           = m_g
    tank_conditions.fuel_mass[:,0]             = m_l
    # fuel_volume/ullage_volume are algebraic outputs (V_l = m_l/rho_l(T_l)),
    # not integrated states -- see y0 construction above.
    tank_conditions.ullage_volume[:,0]         = diag['V_g']
    tank_conditions.fuel_volume[:,0]           = diag['V_l']
    tank_conditions.ullage_temperature[:,0]    = T_g
    tank_conditions.fuel_temperature[:,0]      = T_l
    tank_conditions.pressure[:,0]              = diag['P']
    tank_conditions.vent_rate[:,0]             = diag['m_dot_vent']
    tank_conditions.boil_off_flow_rate[:,0]    = diag['m_dot_bo_final']
    tank_conditions.heater_power[:,0]          = diag['heater_power']
    tank_conditions.refuel_mass_flow_rate[:,0] = refuel_pts_eff
    # Diagnostic-only (not fed back into any dynamics): environmental heat leak
    # into the liquid- and ullage-contacted portions of the shell, for checking
    # actual modeled heat leak against tank.design_total_heat_transfer.
    tank_conditions.environmental_heat_leak_liquid[:,0] = diag['Q_e_l']
    tank_conditions.environmental_heat_leak_ullage[:,0] = diag['Q_e_g']
    tank_conditions.temperature_out_of_range[:,0]       = diag['T_out_of_range']
    tank_conditions.liquid_thermal_floor_active[:,0]    = diag['m_l_thermal_floored']
    tank_conditions.liquid_mass_floor_blend[:,0]        = diag['m_barrier_blend']
    tank_conditions.liquid_availability_gate[:,0]       = diag['m_dot_avail']
    tank_conditions.heater_saturated[:,0]               = diag['heater_saturated']
    tank_conditions.refuel_volume_capped[:,0]           = float(volume_capped)

    # Net mass leaving the tank system (engine offtake + vented boil-off - refuel inflow)
    tank_conditions.mass_flow_rate[:,0]         = diag['m_dot_l_engine'] + diag['m_dot_vent'] - refuel_pts_eff
    tank_conditions.outputs.power.chemical[:,0] = diag['m_dot_l_engine'] * fuel.lower_heating_value

    stored_results_flag = True
    stored_source_tag    = tank.tag
    result = tank_conditions.inputs, tank_conditions.outputs, stored_results_flag, stored_source_tag
    tank_conditions._boil_off_solve_cache = (tuple(np.array(k, copy=True) for k in cache_key), result)
    return result


def _tank_state_rates(tank, fuel, R_specific, m_g, m_l, T_g, T_l,
                       chemical_power, T_env, nu_air, Pr_air, k_air, m_dot_refuel,
                       T_g_lo, T_g_hi, T_l_lo, T_l_hi, P_lo, P_hi, max_heater_power):
    """Evaluates the tank's 4-state ODE right-hand side plus diagnostic outputs
    (pressure, volume, boil-off/vent split, heater power) at one or more
    instants. All state/input arguments are 1-D arrays of the same length --
    length 1 for a single adaptive-integrator evaluation (``rhs`` above), or
    length n_nodes for the final diagnostic pass at the mission's own
    collocation points. This is the same physics ``compute_cryogenic_tank_performance``
    always used, factored out so it can be called both as an ODE right-hand
    side and as a one-shot diagnostic evaluator without duplicating the model.

    fuel_volume/ullage_volume (V_l/V_g) are NOT states here -- V_l = m_l/rho_l(T_l)
    is computed fresh from the current mass/temperature every call, and
    V_g = gross_volume - V_l, rather than being integrated independently.
    An earlier 6-state version integrated volume directly and was replaced
    after confirming the two volume states could drift out of consistency
    with mass during completely ordinary draining, not just as a near-empty
    edge case (see git history for that version).
    Because -P*dV_l appears in the liquid energy balance, and dV_l is itself a
    function of dT_l through the chain rule on V_l(m_l, T_l), dT_l is solved
    in closed form as a linear equation rather than substituted naively.

    Clamps m to a small positive floor and T to the property table's valid
    range before any lookup or EOS evaluation: Radau's internal trial steps
    and Jacobian estimation can transiently query states outside the
    physically valid domain (e.g. slightly negative mass), and the property
    tables raise instead of extrapolating. Only the *lookups* are clamped --
    the temperature barrier term below still sees the raw (unclamped) T so
    the dynamics themselves resist leaving the valid domain rather than the
    residual just going flat there.
    """
    m_g_c = np.clip(m_g, 1e-6, None)
    m_l_c = np.clip(m_l, 1e-6, None)
    T_g_c = np.clip(T_g, T_g_lo, T_g_hi)
    T_l_c = np.clip(T_l, T_l_lo, T_l_hi)

    # ------------------------------------------------------------------
    #  Liquid volume/density and their derivative, needed both for V_g below
    #  and for the -P*dV_l coupling in the liquid energy balance further down.
    #  drho_l_dT_l is a centered finite difference (clipped to the table's
    #  valid range on both sides) -- the property tables don't expose an
    #  analytic derivative, and rho_l(T) is smooth enough over this scale for
    #  a simple FD to be adequate.
    # ------------------------------------------------------------------
    rho_l = fuel.cryogen_properties(T_l_c, "Density (kg/m3)", phase='liquid')
    V_l_c = m_l_c / rho_l
    V_g_c = np.clip(tank.volume_properties.gross_volume - V_l_c, 1e-6, None)

    dT_fd   = 0.01  # K
    T_plus  = np.clip(T_l_c + dT_fd, T_l_lo, T_l_hi)
    T_minus = np.clip(T_l_c - dT_fd, T_l_lo, T_l_hi)
    rho_l_plus  = fuel.cryogen_properties(T_plus,  "Density (kg/m3)", phase='liquid')
    rho_l_minus = fuel.cryogen_properties(T_minus, "Density (kg/m3)", phase='liquid')
    drho_l_dT_l = (rho_l_plus - rho_l_minus) / np.clip(T_plus - T_minus, 1e-6, None)

    # ------------------------------------------------------------------
    #  Engine fuel offtake demand (same derivation the base explicit-
    #  integration fuel tank model uses: distributor's already-computed
    #  chemical power demand, split by power_split_ratio)
    # ------------------------------------------------------------------
    m_dot_l_engine = chemical_power / fuel.lower_heating_value

    # ------------------------------------------------------------------
    #  Ullage pressure: ideal gas corrected by a real-gas compressibility
    #  factor Z(T_g) = P_sat_table(T_g)/(rho_sat_table(T_g)*R*T_g), taken from
    #  the saturation table's own real vapor density/pressure pair. A bare
    #  ideal-gas EOS (Z=1) underpredicts pressure substantially near
    #  saturation (confirmed empirically: it drove persistent, unphysical
    #  boil-off because the ullage always reads as under-pressurized relative
    #  to its own bulk temperature), so this correction anchors the ullage
    #  EOS to the table's real state at the current ullage temperature.
    # ------------------------------------------------------------------
    Z         = fuel.compressibility_factor(T_g_c, phase='vapor')
    P         = Z * (m_g_c / V_g_c) * R_specific * T_g_c   # Pa
    P_clamped = np.clip(P / Units.MPa, P_lo, P_hi)          # table pressure column is in MPa
    T_int     = fuel.saturation_temperature(P_clamped)

    # ------------------------------------------------------------------
    #  Interface geometry from the current liquid volume
    # ------------------------------------------------------------------
    L_int, A_int, A_wet_frac = compute_interface_geometric_properties(tank, V_l_c)

    # ------------------------------------------------------------------
    #  Bulk liquid/vapor properties at the current bulk temperatures
    # ------------------------------------------------------------------
    cp_liq = fuel.cryogen_properties(T_l_c, "Cp (J/g*K)",          phase='liquid') * 1000.0
    mu_liq = fuel.cryogen_properties(T_l_c, "Viscosity (Pa*s)",    phase='liquid')
    k_liq  = fuel.cryogen_properties(T_l_c, "Therm. Cond. (W/m*K)", phase='liquid')

    rho_g = fuel.cryogen_properties(T_g_c, "Density (kg/m3)",     phase='vapor')
    cp_g  = fuel.cryogen_properties(T_g_c, "Cp (J/g*K)",          phase='vapor') * 1000.0  # Prandtl-number convection correlation (Q_gas_to_int) uses Cp by convention
    cv_g  = fuel.cryogen_properties(T_g_c, "Cv (J/g*K)",          phase='vapor') * 1000.0  # ullage energy balance (Eq. 16) uses Cv, a fixed-volume control mass
    mu_g  = fuel.cryogen_properties(T_g_c, "Viscosity (Pa*s)",    phase='vapor')
    k_g   = fuel.cryogen_properties(T_g_c, "Therm. Cond. (W/m*K)", phase='vapor')

    # Bulk ullage/liquid enthalpy and internal energy, at each region's own bulk
    # temperature -- NOT the interface/saturation temperature. Per Adler & Martins
    # (2025) Eq. 8/20: boil-off gas "enters the interface as a liquid with the LH2's
    # specific enthalpy and exits the interface as a gas with the ullage's specific
    # enthalpy" -- i.e. h_g/u_g at T_g, h_l/u_l at T_l, matching the AST paper's
    # unsubscripted h_g/u_g, h_l/u_l in Eq. 16-17 (T_int is used only for the
    # interface heat-transfer driving temperature difference below).
    h_g = fuel.cryogen_properties(T_g_c, "Enthalpy (kJ/kg)",        phase='vapor')  * 1000.0
    h_l = fuel.cryogen_properties(T_l_c, "Enthalpy (kJ/kg)",        phase='liquid') * 1000.0
    u_g = fuel.cryogen_properties(T_g_c, "Internal Energy (kJ/kg)", phase='vapor')  * 1000.0
    u_l = fuel.cryogen_properties(T_l_c, "Internal Energy (kJ/kg)", phase='liquid') * 1000.0

    # ------------------------------------------------------------------
    #  Interface heat transfer (natural convection, bulk liquid/gas -> interface)
    # ------------------------------------------------------------------
    Q_l_i = Q_liq_to_int(T_l_c, T_int, A_int, L_int, rho_l, cp_liq, mu_liq, k_liq)
    Q_g_i = Q_gas_to_int(T_g_c, T_int, A_int, L_int, rho_g, cp_g,  mu_g,  k_g)

    # ------------------------------------------------------------------
    #  Environment heat leak through the already-sized wall + insulation,
    #  evaluated at actual flight ambient conditions and split between the
    #  liquid- and ullage-contacted portions of the shell by current
    #  wetted-area fraction (Churchill/conduction-network physics shared
    #  with the design-time insulation sizing solve)
    # ------------------------------------------------------------------
    T_env  = np.atleast_1d(T_env)
    nu_air = np.atleast_1d(nu_air)
    Pr_air = np.atleast_1d(Pr_air)
    k_air  = np.atleast_1d(k_air)
    alpha_air = nu_air / Pr_air

    t_ins     = tank.insulation.thickness
    k_mat     = tank.inner_structure.material.thermal_conductivity
    k_ins_mat = tank.insulation.material.thermal_conductivity

    n_nodes   = len(T_env)
    Q_env_liq = np.zeros(n_nodes)
    Q_env_gas = np.zeros(n_nodes)

    if tank.geometry_type == 'cylindrical':
        ro = tank.inner_structure.diameters.external / 2
        ri = tank.inner_structure.diameters.internal / 2
        li = tank.inner_structure.lengths.internal
        for i in range(n_nodes):
            _, Q_env_liq[i] = compute_cryogenic_tank_heat_leak(
                t_ins, T_env[i], T_l_c[i], k_mat, k_ins_mat, k_air[i], nu_air[i], alpha_air[i], Pr_air[i], ro, ri, li)
            _, Q_env_gas[i] = compute_cryogenic_tank_heat_leak(
                t_ins, T_env[i], T_g_c[i], k_mat, k_ins_mat, k_air[i], nu_air[i], alpha_air[i], Pr_air[i], ro, ri, li)
    else:
        # Conformal/prismatic tanks: same outer convection/radiation-vs-conduction
        # heat-leak model as the cylindrical case, using flat-plate free-convection
        # correlations per face orientation instead of cylinder/sphere correlations
        # (compute_cryogenic_tank_heat_leak_cuboid) -- shared with the design-time
        # insulation sizing solve (compute_cryogenic_conformal_tank_volume.py).
        l_o = tank.inner_structure.lengths.external
        w_o = tank.inner_structure.widths.external
        h_o = tank.inner_structure.heights.external
        th  = tank.inner_structure.thickness
        for i in range(n_nodes):
            _, Q_env_liq[i] = compute_cryogenic_tank_heat_leak_cuboid(
                t_ins, T_env[i], T_l_c[i], k_mat, k_ins_mat, k_air[i], nu_air[i], alpha_air[i], Pr_air[i], l_o, w_o, h_o, th)
            _, Q_env_gas[i] = compute_cryogenic_tank_heat_leak_cuboid(
                t_ins, T_env[i], T_g_c[i], k_mat, k_ins_mat, k_air[i], nu_air[i], alpha_air[i], Pr_air[i], l_o, w_o, h_o, th)

    Q_e_l = Q_env_liq * A_wet_frac
    Q_e_g = Q_env_gas * (1 - A_wet_frac)

    # ------------------------------------------------------------------
    #  Natural boil-off from interfacial heat transfer
    # ------------------------------------------------------------------
    m_dot_bo = (Q_l_i + Q_g_i) / (h_g - h_l)

    # Liquid-availability gate: once the tank runs (near) empty there's no liquid
    # left to boil, but nothing upstream enforces that on its own -- m_l/V_l feed
    # a root-solved interface geometry (compute_interface_geometric_properties)
    # that keeps returning a small nonzero interface area/heat-transfer coefficient
    # even as m_l_c bottoms out at its 1e-6 floor, so m_dot_bo doesn't fall to zero
    # by itself. Confirmed directly: a long ground-dormancy segment on an
    # already near-empty tank produced growing pressure/vent/boil-off oscillations
    # and eventually integrated to a negative fuel_mass. Smoothed (not a hard
    # cutoff) for the same differentiability reason as the complementarity
    # relaxation below; the floor/width scale with the tank's own design capacity
    # so this only activates once genuinely (near) empty, not during ordinary
    # operation.
    #
    # Gated on the *raw* m_l, not the clipped m_l_c: m_l_c is floored at a fixed
    # 1e-6 kg for property lookups, so once the raw state drifts past empty it no
    # longer tracks how empty the tank actually is -- a gate built on m_l_c would
    # freeze at whatever small residual rate that floor implies and never fully
    # shut off, letting the raw (unclamped) state keep creeping further negative
    # instead of settling.
    m_l_floor   = 1e-3 * tank.design_full_liquid_mass
    m_avail_eps = 5e-4 * tank.design_full_liquid_mass
    m_dot_avail = 0.5 * (1.0 + (m_l - m_l_floor) / np.sqrt((m_l - m_l_floor)**2 + m_avail_eps**2))
    m_dot_bo    = m_dot_bo * m_dot_avail

    # Net ullage regulation flow (heater-driven boil-off if positive, vent if
    # negative), computed explicitly from a finite-gain feedback law on the
    # pressure error -- NOT an exact/instantaneous constraint. tau
    # (tank.pressure_regulation_time_constant) is the characteristic response
    # time of the real heater/vent system; V_g/(R_specific*T_g) converts a
    # pressure error into the ullage mass needed to correct it (ideal-gas EOS),
    # and dividing by tau turns that into a rate. As tau -> 0 this recovers an
    # exact P=design_pressure constraint; a finite tau keeps the regulation
    # response bounded (matching the finite heater authority both Adler &
    # Martins (2025) and the AST paper describe) instead of demanding the
    # system instantly cancel every deviation.
    tau         = tank.pressure_regulation_time_constant
    m_dot_reg   = (V_g_c / (R_specific * T_g_c * tau)) * (tank.design_pressure - P)

    # Smoothed complementarity relaxation (replaces a hard np.where threshold):
    # m_dot_reg > 0 -> active-heater boil-off duty; m_dot_reg < 0 -> venting.
    # m_dot_vent is computed as its own smoothed max(-m_dot_reg, 0) -- not by
    # subtracting m_dot_heater_boi from m_dot_reg -- so gating heater duty by
    # liquid availability below doesn't also suppress ordinary venting of
    # already-present ullage gas, which needs no liquid at all.
    eps               = 1e-6  # kg/s, smoothing width
    m_dot_heater_boi  = 0.5 * ( m_dot_reg + np.sqrt(m_dot_reg**2 + eps**2)) * m_dot_avail
    m_dot_vent        = 0.5 * (-m_dot_reg + np.sqrt(m_dot_reg**2 + eps**2))

    # Heater power split, per Adler & Martins (2025) Eq. 13/17/24: only a
    # fraction eta_h of the heater's total power goes directly to flash-
    # boiling liquid at the interface (the m_dot_heater_boi duty above); the
    # remaining (1 - eta_h) fraction instead raises the bulk liquid
    # temperature (their Eq. 17 Q_h(1-eta_h) term) rather than boiling it
    # outright. m_dot_heater_boi*(h_g-h_l) is therefore only the eta_h-share
    # of the total heater power, not the total.
    # eta_h=0.0 is a valid setting (heater only warms the bulk liquid, no direct
    # flash-boiling) -- guarded against 0/0 here since m_dot_heater_boi is itself
    # 0 whenever there's no regulation demand, which is most of a well-regulated
    # mission. The physical limit there is 0 required power, not NaN.
    eta_h                 = tank.heater_direct_boiloff_fraction
    heater_power_uncapped = m_dot_heater_boi * (h_g - h_l) / np.maximum(eta_h, 1e-9)

    # Cap at the real heater's rated power (max_heater_power) -- m_dot_reg has
    # no actuator limit of its own, so a large pressure error can otherwise
    # demand orders of magnitude more power than a real electric heater can
    # deliver. Rescale m_dot_heater_boi/m_dot_bo_final consistently so the
    # capped heater_power stays the actual number driving both mass and
    # energy balances, not just a clipped diagnostic. heater_saturated is
    # exported (not just clamped silently) per RCAIDE policy against hidden
    # floors/patches -- a run spending real mission time saturated is telling
    # the designer/optimizer the heater (or tank insulation) is undersized,
    # not something to paper over invisibly.
    heater_power      = np.minimum(heater_power_uncapped, max_heater_power)
    heater_saturated  = heater_power_uncapped > max_heater_power
    m_dot_heater_boi  = heater_power * eta_h / (h_g - h_l)
    m_dot_bo_final    = m_dot_bo + m_dot_heater_boi
    Q_h_to_liquid     = heater_power * (1 - eta_h)

    # ------------------------------------------------------------------
    #  Mass / volume / energy balances (the ODE right-hand side)
    # ------------------------------------------------------------------
    dm_g = m_dot_bo_final - m_dot_vent
    # Incoming fuel assumed thermally equilibrated with the tank's own bulk liquid,
    # so no separate enthalpy term is needed here (see compute_cryogenic_tank_performance's
    # docstring for the refuel simplification this assumes).
    dm_l = -m_dot_bo_final - m_dot_l_engine + m_dot_refuel

    # Liquid-mass floor: m_dot_avail above only throttles boil-off, so any other
    # residual net outflow (leftover boil-off tail, or engine draw exceeding what's
    # actually left) could still walk m_l slightly negative over a long enough
    # segment. This adds a real restoring inflow once m_l dips below its floor,
    # reusing tau (the tank's own pressure-regulation response time) rather than
    # inventing a new time constant, since m_l has no physical business going
    # negative regardless of why it was trying to.
    #
    # Deliberately compactly supported (a clipped smoothstep), NOT the same
    # sqrt-relaxation used elsewhere in this function (e.g. the temperature
    # barrier below): that form is only asymptotically small far from the
    # boundary, never exactly zero, and confirmed directly to leak a tiny
    # continuous mass source into every ordinary (nowhere-near-empty) tank --
    # enough to trip run_case's mass-conservation check in the VnV suite. Fine
    # for temperature, which has no conservation law being checked against it;
    # not fine here.
    m_barrier_blend = np.clip((m_l_floor + m_avail_eps - m_l) / (2 * m_avail_eps), 0.0, 1.0)
    m_barrier_blend = m_barrier_blend**2 * (3 - 2 * m_barrier_blend)  # smoothstep
    dm_l = dm_l + m_barrier_blend * (m_l_floor - m_l) / tau

    # V_l = m_l/rho_l(T_l) is algebraic (see docstring), so dV_l is its TOTAL
    # derivative by the chain rule:
    #   dV_l = dm_l/rho_l - (m_l/rho_l^2)*drho_l_dT_l*dT_l
    # dV_l appears in dT_l's own energy balance via the -P*dV_l work term, so
    # substituting the line above turns dT_l into a linear equation in itself
    # rather than something we can evaluate directly -- solved in closed form:
    #   dT_l*(m_l_thermal*cp_liq) = RHS0 + P*(m_l_c/rho_l^2)*drho_l_dT_l*dT_l
    #   dT_l = RHS0 / (m_l_thermal*cp_liq - P*(m_l_c/rho_l^2)*drho_l_dT_l)
    # where RHS0 is everything else (the dV_l chain-rule mass-flow term folded
    # in, dT_l-dependence pulled out). See near-empty thermal-mass floor note
    # below for why m_l_thermal (not m_l_c) is used as the denominator's mass.
    m_l_thermal = np.maximum(m_l_c, m_l_floor)
    A_thermal   = m_l_thermal * cp_liq
    B_coupling  = P * m_l_c / rho_l**2 * drho_l_dT_l
    RHS0        = -Q_l_i + Q_e_l + Q_h_to_liquid - P * dm_l / rho_l + dm_l * (h_l - u_l)
    dT_l = RHS0 / (A_thermal - B_coupling)

    dV_l = dm_l / rho_l - (m_l_c / rho_l**2) * drho_l_dT_l * dT_l
    dV_g = -dV_l

    dT_g = (-Q_g_i + Q_e_g - P * dV_g + dm_g * (h_g - u_g)) / (m_g_c * cv_g)

    # Soft restoring term outside the property table's range. The table's edges are
    # not arbitrary data cutoffs -- e.g. hydrogen's upper bound is its actual critical
    # temperature, above which there is no distinct saturated liquid/vapor at all -- so
    # clamping only the property *lookups* (above) leaves the dynamics themselves flat
    # (insensitive) once T exceeds the table, and an integrator can wander onto a
    # self-consistent but unphysical "runaway" branch out there. This adds a real
    # restoring force so the dynamics themselves resist leaving the valid domain,
    # smoothed the same way as the boil-off/vent split above.
    T_barrier_rate = 0.05  # 1/s, restoring-rate strength per K of excess
    T_eps          = 0.1   # K, smoothing width
    dT_g = dT_g + T_barrier_rate * _soft_temperature_barrier(np.atleast_1d(T_g), T_g_lo, T_g_hi, T_eps)
    dT_l = dT_l + T_barrier_rate * _soft_temperature_barrier(np.atleast_1d(T_l), T_l_lo, T_l_hi, T_eps)

    # Observability, not correction: these flags/values are exported as-is so a
    # run relying on the floors/barriers above to stay numerically alive is
    # visible in the output, not silently absorbed -- per RCAIDE policy against
    # hidden patches that hide where the model actually broke down.
    T_out_of_range      = (T_g < T_g_lo) | (T_g > T_g_hi) | (T_l < T_l_lo) | (T_l > T_l_hi)
    m_l_thermal_floored = m_l_c < m_l_floor

    derivatives = (dm_g, dm_l, dT_g, dT_l)
    diagnostics = dict(P=P, m_dot_vent=m_dot_vent, m_dot_bo_final=m_dot_bo_final,
                        heater_power=heater_power, m_dot_l_engine=m_dot_l_engine,
                        Q_e_l=Q_e_l, Q_e_g=Q_e_g,
                        # V_l/V_g are algebraic outputs here (not states) -- returned so
                        # the caller can store them without recomputing the density lookup.
                        V_l=V_l_c, V_g=V_g_c,
                        # Diagnostic-only additions for tracing dT_l's individual terms:
                        Q_l_i=Q_l_i, P_dV_l_term=-P * dV_l, enthalpy_flux_l=dm_l * (h_l - u_l),
                        Q_h_to_liquid=Q_h_to_liquid, dT_l=dT_l, rho_l=rho_l, m_l_thermal=m_l_thermal,
                        # Observability of where a run is leaning on a floor/cap/barrier:
                        T_out_of_range=T_out_of_range, m_l_thermal_floored=m_l_thermal_floored,
                        m_barrier_blend=m_barrier_blend, m_dot_avail=m_dot_avail,
                        heater_saturated=heater_saturated)
    return derivatives, diagnostics


def _soft_temperature_barrier(T, T_lo, T_hi, eps):
    """Smoothed restoring direction, 0 inside [T_lo, T_hi], -1 above T_hi, +1 below
    T_lo (scaled by T_barrier_rate at the call site to a rate)."""
    above = -0.5 * ((T - T_hi) + np.sqrt((T - T_hi)**2 + eps**2))
    below = 0.5 * ((T_lo - T) + np.sqrt((T_lo - T)**2 + eps**2))
    return above + below


# ----------------------------------------------------------------------------------------------------------------------
#  Interface heat transfer: bulk liquid / bulk gas -> liquid-vapor interface
#
#  Natural convection via a Grashof-Prandtl correlation (Ring); generic across any
#  cryogen -- no fluid-specific constants.
#
#  |T_bulk - T_int| is smoothed as sqrt(dT**2 + DELTA_T**2) rather than np.abs(dT).
#  The fractional exponent n=0.25 on GrPr (proportional to |dT|) gives alpha, and
#  therefore Q, an infinite *derivative* exactly at dT=0 even though the value stays
#  finite and continuous there -- confirmed directly: a Jacobian eigenvalue of
#  ~-1.5e8 (vs. ~1e-3 for every other mode) appears exactly when a trajectory has the
#  bulk temperature cross T_int (an ordinary boiling<->condensing direction change,
#  not any special state of the fluid), which stalls any integrator needing
#  derivative/Jacobian information. This regularization matches the value away from
#  dT=0 to within DELTA_T and keeps the derivative bounded everywhere, the same
#  smoothing technique already used for the boil-off/vent split and the temperature
#  barrier elsewhere in this file.
# ----------------------------------------------------------------------------------------------------------------------
DELTA_T = 0.1  # K, smoothing width for the interface temperature-difference singularity

def Q_liq_to_int(T_liq, T_int, A_int, L_int, rho_l, cp_l, mu_l, k_l, C=0.27, n=0.25):
    beta_l = 1.0 / T_liq
    dT = T_liq - T_int
    GrPr = (L_int**3 * rho_l**2 * 9.81 * beta_l *
            np.sqrt(dT**2 + DELTA_T**2) * cp_l) / (mu_l * k_l + 1e-12)
    alpha = C * (k_l / L_int) * (GrPr**n)
    return alpha * A_int * dT


def Q_gas_to_int(T_g, T_int, A_int, L_int, rho_g, cp_g, mu_g, k_g, C=0.27, n=0.25):
    beta_g = 1.0 / T_g
    dT = T_g - T_int
    GrPr = (L_int**3 * rho_g**2 * 9.81 * beta_g *
            np.sqrt(dT**2 + DELTA_T**2) * cp_g) / (mu_g * k_g + 1e-12)
    alpha = C * (k_g / L_int) * (GrPr**n)
    return alpha * A_int * dT


# ----------------------------------------------------------------------------------------------------------------------
#  Liquid height and interface geometry for a rounded-end cylindrical tank
#  (cylinder + hemispherical caps), from the current liquid volume
# ----------------------------------------------------------------------------------------------------------------------
def _liquid_height_residual(h, r, l, v):
    return l * (r**2 * np.arccos((r - h) / r) - (r - h) * np.sqrt(2 * r * h - h**2)) + \
        (np.pi / 3) * h**2 * (3 * r - h) - v


def compute_liquid_height(r, l, v_array):
    """Vectorized: solves for liquid height given an array of liquid volumes."""
    v_array  = np.clip(np.atleast_1d(np.asarray(v_array, dtype=float)), 0.0, None)
    v_full   = np.pi * r**2 * l + (4 / 3) * np.pi * r**3
    v_clipped = np.clip(v_array, 1e-9, v_full - 1e-9)
    h_array = np.fromiter(
        (brentq(_liquid_height_residual, 1e-9, 2 * r - 1e-9, args=(r, l, vv)) for vv in v_clipped),
        dtype=float
    )
    return h_array


def compute_interface_geometric_properties(tank, v_l):
    """
    Returns the interface length/area (liquid-vapor free surface) and the
    fraction of the tank's shell surface currently wetted by liquid, from the
    current liquid volume.

    Cylindrical tanks (cylinder + hemispherical caps) use the rounded-cylinder
    liquid-height solve below. Conformal/prismatic tanks are sized as a box
    (``inner_structure.lengths/widths/heights.internal``, no diameters) and are
    treated as a simple flat-bottomed slab -- height is a direct h = V_l/(l*w)
    (no root solve needed), the free surface is the constant top area l*w, and
    the wetted-area fraction is the liquid height fraction of the box.
    """
    if tank.geometry_type == 'cylindrical':
        r_in = tank.inner_structure.diameters.internal / 2
        l_in = tank.inner_structure.lengths.internal

        h = compute_liquid_height(r_in, l_in, v_l)

        lamda = h / (2 * r_in)
        L_int = 4 * r_in * np.sqrt(lamda - lamda**2)
        A_int = (np.pi * 2 * r_in * L_int**2 / 4) + L_int * l_in

        A_wet_frac = np.arccos(np.clip((r_in - h) / r_in, -1.0, 1.0)) / np.pi

        return L_int, A_int, A_wet_frac

    else:
        l_in = tank.inner_structure.lengths.internal
        w_in = tank.inner_structure.widths.internal
        H_in = tank.inner_structure.heights.internal

        v_l   = np.clip(np.atleast_1d(np.asarray(v_l, dtype=float)), 0.0, l_in * w_in * H_in)
        h     = v_l / (l_in * w_in)
        L_int = np.sqrt(l_in * w_in) * np.ones_like(h)
        A_int = l_in * w_in * np.ones_like(h)
        A_wet_frac = np.clip(h / H_in, 0.0, 1.0)

        return L_int, A_int, A_wet_frac
