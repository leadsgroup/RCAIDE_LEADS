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
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_tank_heater_design_power import compute_cryogenic_tank_heater_design_power
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.cryogenic_tank_rates_kernel import cryogenic_tank_rates, cryogenic_tank_rates_tables, cryogenic_tank_node_diagnostics, soft_temperature_barrier, liquid_height_residual, DELTA_T
from RCAIDE.Framework.Core.Physical_Constants import UNIVERSAL_GAS_CONSTANT
from RCAIDE.Framework.Core import Units

# Skips re-solving when a fsolve probe couldn't have changed this tank's inputs.
def cache_key_matches(cached, current):
    if cached is None or len(cached) != len(current):
        return False
    # equal_nan=True since refuel_target_mass is NaN outside Refuel segments.
    return all(np.array_equal(a, b, equal_nan=True) for a, b in zip(cached, current))

# Tank condition fields written by a solve; a cache hit restores them since segment initialization resets them in place
SOLVED_FIELDS = ('ullage_mass', 'fuel_mass', 'ullage_volume', 'fuel_volume', 'ullage_temperature', 'fuel_temperature',
                 'pressure', 'vent_rate', 'boil_off_flow_rate', 'heater_power', 'refuel_mass_flow_rate',
                 'environmental_heat_leak_liquid', 'environmental_heat_leak_ullage', 'temperature_out_of_range',
                 'liquid_thermal_floor_active', 'fuel_deficit_rate', 'liquid_availability_gate',
                 'heater_saturated', 'refuel_volume_capped', 'mass_flow_rate')

def store_tank_solve(tank_conditions, cache_key, result):
    """Caches a tank solve together with every condition array it wrote."""
    solved = {field: np.array(tank_conditions[field], copy=True) for field in SOLVED_FIELDS}
    solved['chemical_power'] = np.array(tank_conditions.outputs.power.chemical, copy=True)
    tank_conditions._boil_off_solve_cache = (tuple(np.array(k, copy=True) for k in cache_key), result, solved)
    return result

def restore_tank_solve(tank_conditions, solved):
    """Writes a cached solve's condition arrays back into the tank conditions."""
    for field in SOLVED_FIELDS:
        tank_conditions[field][:,:] = solved[field]
    tank_conditions.outputs.power.chemical[:,:] = solved['chemical_power']
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Cryogenic Tank In-Flight Boil-Off Performance
# ----------------------------------------------------------------------------------------------------------------------
def compute_cryogenic_tank_performance(tank, state, network, rtol=1e-8, atol=1e-10, method='LSODA'):
    """
    Two-phase lumped-parameter boil-off model of a cryogenic tank, after Adler & Martins (2025).

    The four states (ullage and liquid mass and temperature) are integrated across the segment with
    scipy's solve_ivp, driven by the engine fuel demand and freestream conditions interpolated between the
    segment's control points, and sampled back at those points. They are not mission solver unknowns. Fluid
    properties come from the propellant's saturated-property tables, with a compressibility factor for the
    ullage.

    The heater adds boil-off below minimum_operating_pressure and the tank vents above design_pressure.
    Bulk boiling and cloud condensation keep the liquid and ullage at or on the correct side of saturation.
    Liquid outflows stop at empty, and demand the tank cannot meet is reported as fuel_deficit_rate. Each tank
    uses its fixed power_split_ratio share for the segment; an empty tank's unmet demand is not reassigned to
    other tanks on the same distributor.

    method, rtol and atol default to LSODA, 1e-8 and 1e-10. The mission solver differentiates through this
    solve by finite differences, so integration error must stay well below those perturbations.

    Notes
    -----
    A Ground.Refuel segment fills at refuel_mass_flow_rate until the liquid reaches refuel_target_mass or
    the volume cutoff, then holds. The cutoff is lowered until the liquid stays within net_volume for the
    whole segment. Incoming fuel is at the bulk liquid temperature. The heater responds with time constant
    pressure_regulation_time_constant; its thermal inertia is not modeled.
    """
    tag             = tank.tag
    tank_conditions = state.conditions.energy.sources[tag]
    fuel            = tank.fuel

    t = np.ravel(state.numerics.time.control_points)

    (T_l_lo, T_l_hi), (P_lo, P_hi) = fuel.property_table_range(phase='liquid')
    (T_g_lo, T_g_hi), _            = fuel.property_table_range(phase='vapor')
    R_specific = UNIVERSAL_GAS_CONSTANT / fuel.molecular_weight  # J/(kg*K)

    # Driving-input trajectories at the mission's own (sparse) collocation points --
    # np.interp below resamples these onto the adaptive stepper's intermediate times.
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
    if cached is not None and cache_key_matches(cached[0], cache_key):
        restore_tank_solve(tank_conditions, cached[2])
        return cached[1]

    # Heater cap: Heater.design_power when set, else sized to the takeoff pressurant demand, else 10x design heat leak
    max_heater_power = compute_cryogenic_tank_heater_design_power(tank, network) if network is not None else None
    if max_heater_power is None:
        max_heater_power = 10.0 * tank.design_total_heat_transfer
    if network is not None:
        for converter in getattr(network, 'converters', []):
            if isinstance(converter, RCAIDE.Library.Components.Powertrain.Converters.Heater) and \
               getattr(converter, 'assigned_tank', None) == tag and converter.design_power is not None:
                max_heater_power = converter.design_power
                break

    if t.size < 2:
        m_g0, m_l0, T_g0, T_l0 = (np.array([v]) for v in y0)

        _, diag = tank_state_rates(
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
        tank_conditions.fuel_deficit_rate[:,0]         = diag['m_dot_deficit']
        tank_conditions.liquid_availability_gate[:,0]  = diag['m_dot_avail']
        tank_conditions.heater_saturated[:,0]          = diag['heater_saturated']
        tank_conditions.refuel_volume_capped[:,0]      = 0.0

        result = tank_conditions.inputs, tank_conditions.outputs, True, tank.tag
        return store_tank_solve(tank_conditions, cache_key, result)

    duration = t[-1] - t[0]

    # Per-state atol instead of one scalar across mass/temp.
    state_floors = np.array([1e-3, 1e-3, 1.0, 1.0])
    atol_vec = atol * np.maximum(np.abs(y0), state_floors)

    # Compiled right-hand side (cryogenic_tank_rates_kernel); the Python one below only runs when the kernel
    # flags a heat-leak solve that needs its fallback path.
    rate_tables = cryogenic_tank_rates_tables(tank, R_specific, T_g_lo, T_g_hi, T_l_lo, T_l_hi, P_lo, P_hi, max_heater_power)
    t_nodes     = np.ascontiguousarray(t, dtype=float)

    def build_rhs(refuel_pts_local):
        inputs = np.ascontiguousarray(np.vstack([chemical_power_pts, T_env_pts, nu_air_pts, Pr_air_pts, k_air_pts, refuel_pts_local]), dtype=float)
        def rhs(t_i, y):
            rates, _, status = cryogenic_tank_rates(t_i, np.asarray(y, dtype=float), t_nodes, inputs, *rate_tables)
            if status == 0:
                return rates
            m_g, m_l, T_g, T_l = y
            chemical_power = np.interp(t_i, t, chemical_power_pts)
            T_env          = np.interp(t_i, t, T_env_pts)
            nu_air         = np.interp(t_i, t, nu_air_pts)
            Pr_air         = np.interp(t_i, t, Pr_air_pts)
            k_air          = np.interp(t_i, t, k_air_pts)
            refuel         = np.interp(t_i, t, refuel_pts_local)
            (dm_g, dm_l, dT_g, dT_l), _ = tank_state_rates(
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
        def liquid_volume(m_l, T_l):
            rho_l = fuel.cryogen_properties(np.atleast_1d(np.clip(T_l, T_l_lo, T_l_hi)), "Density (kg/m3)", phase='liquid')
            return m_l / rho_l

        def solve_refuel(volume_limit):
            """Fills until the mass target or volume_limit is reached, then holds with no fill."""
            def tank_full(t_i, y):
                return y[1] - refuel_target
            tank_full.terminal  = True
            tank_full.direction = 1

            def tank_volume_full(t_i, y):
                return liquid_volume(y[1], y[3])[0] - volume_limit
            tank_volume_full.terminal  = True
            tank_volume_full.direction = 1

            rhs_hold = build_rhs(np.zeros_like(refuel_pts))
            if liquid_volume(y0[1], y0[3])[0] >= volume_limit:
                sol = solve_ivp(rhs_hold, (t[0], t[-1]), y0, method=method, t_eval=t, rtol=rtol, atol=atol_vec)
                if not sol.success:
                    raise RuntimeError(f"Cryogenic tank '{tag}' boil-off IVP failed to integrate: {sol.message}")
                return sol.y, np.zeros_like(refuel_pts), True

            rhs_fill = build_rhs(refuel_pts)
            sol      = solve_ivp(rhs_fill, (t[0], t[-1]), y0, method=method, t_eval=t,
                                 rtol=rtol, atol=atol_vec, events=[tank_full, tank_volume_full])
            if not sol.success:
                raise RuntimeError(f"Cryogenic tank '{tag}' boil-off IVP failed to integrate: {sol.message}")
            fired_events = [i for i, te in enumerate(sol.t_events) if te.size]
            if not fired_events:
                return sol.y, refuel_pts, False

            # Fill reached the target or the volume limit: splice in a zero-fill hold
            event_idx        = fired_events[0]
            t_event, y_event = sol.t_events[event_idx][0], sol.y_events[event_idx][0]
            t_before, t_after = t[t <= t_event], t[t > t_event]
            sol_before = solve_ivp(rhs_fill, (t[0], t_event), y0, method=method, t_eval=t_before, rtol=rtol, atol=atol_vec)
            if not sol_before.success:
                raise RuntimeError(f"Cryogenic tank '{tag}' boil-off IVP failed to integrate (fill): {sol_before.message}")
            y_all = sol_before.y
            if t_after.size:
                sol_after = solve_ivp(rhs_hold, (t_event, t[-1]), y_event, method=method, t_eval=t_after, rtol=rtol, atol=atol_vec)
                if not sol_after.success:
                    raise RuntimeError(f"Cryogenic tank '{tag}' boil-off IVP failed to integrate (hold): {sol_after.message}")
                y_all = np.concatenate([y_all, sol_after.y], axis=1)
            return y_all, np.where(t <= t_event, refuel_pts, 0.0), event_idx == 1

        # Condensation and warming after the fill stops keep expanding the liquid, so the volume cutoff is
        # lowered by the overshoot until the liquid stays within net_volume for the whole segment
        volume_limit = net_volume
        for iteration in range(4):
            y_all, refuel_pts_eff, volume_capped = solve_refuel(volume_limit)
            overshoot = np.max(liquid_volume(y_all[1], y_all[3])) - net_volume
            if overshoot <= 1e-6 * net_volume:
                break
            volume_limit -= overshoot
        m_g, m_l, T_g, T_l = y_all

        # A fill that approaches net_volume without crossing it is still volume limited
        volume_capped = volume_capped or bool(liquid_volume(m_l[-1], T_l[-1])[0] >= net_volume * (1 - 2e-2))
    else:
        if np.isfinite(refuel_target):
            # Already at/above target (e.g. topping off a nearly-full tank) -- don't add fuel.
            refuel_pts = np.zeros_like(refuel_pts)
        rhs = build_rhs(refuel_pts)
        sol = solve_ivp(rhs, (t[0], t[-1]), y0, method=method, t_eval=t, rtol=rtol, atol=atol_vec)
        if not sol.success:
            raise RuntimeError(
                f"Cryogenic tank '{tag}' internal boil-off IVP failed to integrate: {sol.message}")
        m_g, m_l, T_g, T_l = sol.y
        refuel_pts_eff = refuel_pts

    # Recover diagnostic outputs (pressure, vent/boil-off split, heater power) at
    # the collocation points, using their own real (not re-interpolated) inputs.
    inputs_eff = np.ascontiguousarray(np.vstack([chemical_power_pts, T_env_pts, nu_air_pts, Pr_air_pts, k_air_pts, refuel_pts_eff]), dtype=float)
    diag       = cryogenic_tank_node_diagnostics(t_nodes, m_g, m_l, T_g, T_l, inputs_eff, rate_tables)
    if diag is None:
        _, diag = tank_state_rates(
            tank, fuel, R_specific, m_g, m_l, T_g, T_l,
            chemical_power_pts, T_env_pts, nu_air_pts, Pr_air_pts, k_air_pts, refuel_pts_eff,
            T_g_lo, T_g_hi, T_l_lo, T_l_hi, P_lo, P_hi, max_heater_power)

    # ------------------------------------------------------------------
    #  Store outputs
    # ------------------------------------------------------------------
    tank_conditions.ullage_mass[:,0]           = m_g
    tank_conditions.fuel_mass[:,0]             = m_l
    # fuel_volume/ullage_volume are algebraic outputs (V_l = m_l/rho_l(T_l)), not states.
    tank_conditions.ullage_volume[:,0]         = diag['V_g']
    tank_conditions.fuel_volume[:,0]           = diag['V_l']
    tank_conditions.ullage_temperature[:,0]    = T_g
    tank_conditions.fuel_temperature[:,0]      = T_l
    tank_conditions.pressure[:,0]              = diag['P']
    tank_conditions.vent_rate[:,0]             = diag['m_dot_vent']
    tank_conditions.boil_off_flow_rate[:,0]    = diag['m_dot_bo_final']
    tank_conditions.heater_power[:,0]          = diag['heater_power']
    tank_conditions.refuel_mass_flow_rate[:,0] = refuel_pts_eff
    # Diagnostic-only: environmental heat leak into the liquid/ullage-contacted
    # shell, for checking against tank.design_total_heat_transfer.
    tank_conditions.environmental_heat_leak_liquid[:,0] = diag['Q_e_l']
    tank_conditions.environmental_heat_leak_ullage[:,0] = diag['Q_e_g']
    tank_conditions.temperature_out_of_range[:,0]       = diag['T_out_of_range']
    tank_conditions.liquid_thermal_floor_active[:,0]    = diag['m_l_thermal_floored']
    tank_conditions.fuel_deficit_rate[:,0]              = diag['m_dot_deficit']
    tank_conditions.liquid_availability_gate[:,0]       = diag['m_dot_avail']
    tank_conditions.heater_saturated[:,0]               = diag['heater_saturated']
    tank_conditions.refuel_volume_capped[:,0]           = float(volume_capped)

    # Net mass leaving the tank system (engine offtake + vented boil-off - refuel inflow)
    tank_conditions.mass_flow_rate[:,0]         = diag['m_dot_l_engine'] + diag['m_dot_vent'] - refuel_pts_eff
    tank_conditions.outputs.power.chemical[:,0] = diag['m_dot_l_engine'] * fuel.lower_heating_value

    stored_results_flag = True
    stored_source_tag    = tank.tag
    result = tank_conditions.inputs, tank_conditions.outputs, stored_results_flag, stored_source_tag
    return store_tank_solve(tank_conditions, cache_key, result)


def tank_state_rates(tank, fuel, R_specific, m_g, m_l, T_g, T_l,
                     chemical_power, T_env, nu_air, Pr_air, k_air, m_dot_refuel,
                     T_g_lo, T_g_hi, T_l_lo, T_l_hi, P_lo, P_hi, max_heater_power):
    """Tank ODE right-hand side and diagnostics (pressure, volumes, boil-off and vent flows, heater power).
    State and input arguments are 1-D arrays of equal length: one value for an integrator call, one per node
    for the diagnostic pass.

    Liquid and ullage volumes are algebraic, V_l = m_l / rho_l(T_l) and V_g = gross_volume - V_l. Because
    -P dV_l enters the liquid energy balance and dV_l depends on dT_l, dT_l is solved in closed form. Masses
    and temperatures are clamped before property lookups; the temperature barrier still sees the raw value.
    """
    m_g_c = np.clip(m_g, 1e-6, None)
    m_l_c = np.clip(m_l, 1e-6, None)
    T_g_c = np.clip(T_g, T_g_lo, T_g_hi)
    T_l_c = np.clip(T_l, T_l_lo, T_l_hi)

    # Liquid volume/density and their derivative, needed for V_g below and the
    # -P*dV_l coupling further down. drho_l_dT_l is a centered finite difference
    # (the property tables don't expose an analytic derivative).
    rho_l, cp_liq, mu_liq, k_liq, h_l, u_l = fuel.cryogen_property_set(
        T_l_c, ("Density (kg/m3)", "Cp (J/g*K)", "Viscosity (Pa*s)", "Therm. Cond. (W/m*K)", "Enthalpy (kJ/kg)", "Internal Energy (kJ/kg)"), phase='liquid')
    cp_liq = cp_liq * 1000.0
    h_l    = h_l    * 1000.0
    u_l    = u_l    * 1000.0
    V_l_c = m_l_c / rho_l
    V_g_c = np.clip(tank.volume_properties.gross_volume - V_l_c, 1e-6, None)

    dT_fd   = 0.01  # K
    T_plus  = np.clip(T_l_c + dT_fd, T_l_lo, T_l_hi)
    T_minus = np.clip(T_l_c - dT_fd, T_l_lo, T_l_hi)
    rho_l_plus  = fuel.cryogen_properties(T_plus,  "Density (kg/m3)", phase='liquid')
    rho_l_minus = fuel.cryogen_properties(T_minus, "Density (kg/m3)", phase='liquid')
    drho_l_dT_l = (rho_l_plus - rho_l_minus) / np.clip(T_plus - T_minus, 1e-6, None)

    # Engine fuel offtake demand: distributor's chemical power demand, split by power_split_ratio.
    m_dot_l_engine = chemical_power / fuel.lower_heating_value

    # Ullage pressure with compressibility factor Z(T_g) = P_sat / (rho_sat R T_g) from the saturation table;
    # an ideal gas (Z = 1) underpredicts pressure near saturation
    Z         = fuel.compressibility_factor(T_g_c, phase='vapor')
    P         = Z * (m_g_c / V_g_c) * R_specific * T_g_c   # Pa
    P_clamped = np.clip(P / Units.MPa, P_lo, P_hi)          # table pressure column is in MPa
    T_int     = fuel.saturation_temperature(P_clamped)

    # Interface geometry from the current liquid volume. A symmetric pair's stored geometry is one side of it,
    # so each side holds half the liquid and the pair has twice the interface and shell area.
    n_sides = 2 if tank.xz_plane_symmetric else 1
    L_int, A_int, A_wet_frac = compute_interface_geometric_properties(tank, V_l_c / n_sides)
    A_int = A_int * n_sides

    # Bulk vapor properties at the current bulk temperature (liquid ones are computed with rho_l above)
    rho_g, cp_g, cv_g, mu_g, k_g, h_g, u_g = fuel.cryogen_property_set(
        T_g_c, ("Density (kg/m3)", "Cp (J/g*K)", "Cv (J/g*K)", "Viscosity (Pa*s)", "Therm. Cond. (W/m*K)", "Enthalpy (kJ/kg)", "Internal Energy (kJ/kg)"), phase='vapor')
    cp_g = cp_g * 1000.0  # Prandtl-number convection correlation (Q_gas_to_int) uses Cp by convention
    cv_g = cv_g * 1000.0  # ullage energy balance (Eq. 16) uses Cv, a fixed-volume control mass
    h_g  = h_g  * 1000.0
    u_g  = u_g  * 1000.0

    # Bulk ullage/liquid enthalpy and internal energy, at each region's own bulk
    # temperature (not the interface/saturation temperature): per Adler & Martins
    # (2025) Eq. 8/20, boil-off gas enters the interface as liquid at h_l/u_l(T_l)
    # and exits as gas at h_g/u_g(T_g); T_int drives only the interface heat transfer below.

    # Interface heat transfer (natural convection, bulk liquid/gas -> interface)
    Q_l_i = Q_liq_to_int(T_l_c, T_int, A_int, L_int, rho_l, cp_liq, mu_liq, k_liq)
    Q_g_i = Q_gas_to_int(T_g_c, T_int, A_int, L_int, rho_g, cp_g,  mu_g,  k_g)

    # Environment heat leak through the sized wall + insulation, at actual flight
    # ambient conditions, split liquid/ullage by current wetted-area fraction
    # (same conduction-network physics as the design-time insulation sizing solve).
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

    # the heat-leak root solves are scalar; Python floats make each residual evaluation several times cheaper than numpy scalars
    scalar  = lambda v: np.asarray(v, dtype=float).item()
    t_ins_s, k_mat_s, k_ins_s = scalar(t_ins), scalar(k_mat), scalar(k_ins_mat)
    T_env_l, T_l_l, T_g_l     = T_env.tolist(), np.ravel(T_l_c).tolist(), np.ravel(T_g_c).tolist()
    k_air_l, nu_l, alpha_l, Pr_l = k_air.tolist(), nu_air.tolist(), alpha_air.tolist(), Pr_air.tolist()

    if tank.geometry_type == 'cylindrical':
        ro = scalar(tank.inner_structure.diameters.external / 2)
        ri = scalar(tank.inner_structure.diameters.internal / 2)
        li = scalar(tank.inner_structure.lengths.internal)
        for i in range(n_nodes):
            _, Q_env_liq[i] = compute_cryogenic_tank_heat_leak(
                t_ins_s, T_env_l[i], T_l_l[i], k_mat_s, k_ins_s, k_air_l[i], nu_l[i], alpha_l[i], Pr_l[i], ro, ri, li)
            _, Q_env_gas[i] = compute_cryogenic_tank_heat_leak(
                t_ins_s, T_env_l[i], T_g_l[i], k_mat_s, k_ins_s, k_air_l[i], nu_l[i], alpha_l[i], Pr_l[i], ro, ri, li)
    else:
        # Conformal/prismatic tanks: same heat-leak model, using flat-plate
        # free-convection correlations per face instead of cylinder/sphere ones.
        l_o = scalar(tank.inner_structure.lengths.external)
        w_o = scalar(tank.inner_structure.widths.external)
        h_o = scalar(tank.inner_structure.heights.external)
        th  = scalar(tank.inner_structure.thickness)
        for i in range(n_nodes):
            _, Q_env_liq[i] = compute_cryogenic_tank_heat_leak_cuboid(
                t_ins_s, T_env_l[i], T_l_l[i], k_mat_s, k_ins_s, k_air_l[i], nu_l[i], alpha_l[i], Pr_l[i], l_o, w_o, h_o, th)
            _, Q_env_gas[i] = compute_cryogenic_tank_heat_leak_cuboid(
                t_ins_s, T_env_l[i], T_g_l[i], k_mat_s, k_ins_s, k_air_l[i], nu_l[i], alpha_l[i], Pr_l[i], l_o, w_o, h_o, th)

    Q_e_l = n_sides * Q_env_liq * A_wet_frac
    Q_e_g = n_sides * Q_env_gas * (1 - A_wet_frac)

    # Natural boil-off from interfacial heat transfer
    m_dot_bo = (Q_l_i + Q_g_i) / (h_g - h_l)

    # Liquid availability: 1 above the unusable liquid (m_l_floor) plus a smoothing band, exactly 0 at or below it.
    # Every liquid outflow is gated, so an undersized tank stops at empty, conserves mass, and stays within the
    # model's 0-100% fill validity (Adler & Martins 2025, Sec. III); demand it cannot meet is reported as a deficit.
    m_l_floor   = 1e-3 * tank.design_full_liquid_mass
    m_avail_eps = 5e-4 * tank.design_full_liquid_mass
    s_avail     = np.clip((m_l - m_l_floor) / m_avail_eps, 0.0, 1.0)
    m_dot_avail = s_avail**2 * (3 - 2 * s_avail)
    m_dot_bo    = m_dot_bo * m_dot_avail

    # Pressure regulation with a deadband (Adler & Martins 2025, Sec. I): the heater adds boil-off below
    # minimum_operating_pressure, the tank vents above design_pressure, and in between it self-pressurizes.
    # Each flow is a finite-gain law with response time tau, smoothed so the mission Jacobian stays continuous.
    tau              = tank.pressure_regulation_time_constant
    tau_vent         = tank.vent_time_constant if tank.vent_time_constant is not None else tau
    gain             = V_g_c / (R_specific * T_g_c * tau)
    m_dot_heat       = gain * (tank.minimum_operating_pressure - P)
    m_dot_relief     = V_g_c / (R_specific * T_g_c * tau_vent) * (P - tank.design_pressure)
    eps              = 1e-6  # kg/s, smoothing width
    m_dot_heater_boi = 0.5 * (m_dot_heat + np.sqrt(m_dot_heat**2 + eps**2)) * m_dot_avail
    m_dot_vent       = 0.5 * (m_dot_relief + np.sqrt(m_dot_relief**2 + eps**2))

    # Heater power split, Adler & Martins (2025) Eqs. 13, 17 and 24: a fraction eta_h of heater power boils
    # liquid at the interface and the rest warms the liquid. eta_h = 0 is guarded against division by zero.
    eta_h                 = tank.heater_direct_boiloff_fraction
    heater_power_uncapped = m_dot_heater_boi * (h_g - h_l) / np.maximum(eta_h, 1e-9)

    # Cap at the heater's rated power and rescale the heater boil-off to match; heater_saturated reports the cap
    heater_power      = np.minimum(heater_power_uncapped, max_heater_power)
    heater_saturated  = heater_power_uncapped > max_heater_power
    m_dot_heater_boi  = heater_power * eta_h / (h_g - h_l)
    m_dot_bo_final    = m_dot_bo + m_dot_heater_boi
    Q_h_to_liquid     = heater_power * (1 - eta_h)

    # Mass / volume / energy balances (the ODE right-hand side)
    dm_g = m_dot_bo_final - m_dot_vent
    # Incoming fuel assumed thermally equilibrated with the tank's own bulk
    # liquid, so no separate enthalpy term is needed here.
    m_dot_deficit  = m_dot_l_engine * (1 - m_dot_avail)
    m_dot_l_engine = m_dot_l_engine * m_dot_avail
    dm_l = -m_dot_bo_final - m_dot_l_engine + m_dot_refuel

    # dV_l = dm_l/rho_l - (m_l/rho_l^2) drho_l/dT_l dT_l enters the liquid energy balance through -P dV_l, so
    # dT_l = RHS0 / (m_l_thermal cp_liq - P (m_l_c/rho_l^2) drho_l/dT_l); m_l_thermal floors the mass near empty
    m_l_thermal = np.maximum(m_l_c, m_l_floor)
    A_thermal   = m_l_thermal * cp_liq
    B_coupling  = P * m_l_c / rho_l**2 * drho_l_dT_l
    RHS0        = -Q_l_i + Q_e_l + Q_h_to_liquid - P * dm_l / rho_l + dm_l * (h_l - u_l)
    dT_l = RHS0 / (A_thermal - B_coupling)

    dV_l = dm_l / rho_l - (m_l_c / rho_l**2) * drho_l_dT_l * dT_l
    dV_g = -dV_l

    dT_g = (-Q_g_i + Q_e_g - P * dV_g + dm_g * (h_g - u_g)) / (m_g_c * cv_g)

    # Bulk boiling and cloud condensation, Adler & Martins (2025) Eqs. 36-59: mass flows that cancel the
    # pressure falling below the liquid's vapor pressure (bulk boiling) and the ullage temperature falling below
    # its saturation temperature (cloud condensation), each switched on smoothly by a sigmoid (w_s = 100).
    W_S        = 100.0
    P_dot      = P * (dm_g / m_g_c + dT_g / T_g_c - dV_g / V_g_c)
    P_sat_l    = fuel.cryogen_properties(T_l_c, "Pressure (MPa)", phase='liquid') * Units.MPa
    dPsat_dT_l = (fuel.cryogen_properties(T_plus,  "Pressure (MPa)", phase='liquid') -
                  fuel.cryogen_properties(T_minus, "Pressure (MPa)", phase='liquid')) * Units.MPa / np.clip(T_plus - T_minus, 1e-6, None)
    dP_fd      = 1e-3 * P_clamped
    dTsat_dP   = (fuel.saturation_temperature(np.clip(P_clamped + dP_fd, P_lo, P_hi)) -
                  fuel.saturation_temperature(np.clip(P_clamped - dP_fd, P_lo, P_hi))) / (2 * dP_fd * Units.MPa)
    m_dot_bb   = np.abs(V_g_c / (Z * R_specific * T_g_c) * (dPsat_dT_l * dT_l - P_dot))
    m_dot_cc   = np.abs(P * V_g_c / (Z * R_specific * T_g_c**2) * (dTsat_dP * P_dot - dT_g))
    s_bb       = 1.0 / (1.0 + np.exp(np.clip(W_S * (P - P_sat_l) / P_sat_l, -50, 50)))
    s_cc       = 1.0 / (1.0 + np.exp(np.clip(W_S * (T_g_c - T_int) / T_int, -50, 50)))
    m_dot_bb   = s_bb * m_dot_bb * m_dot_avail
    m_dot_cc   = s_cc * m_dot_cc
    m_dot_bbcc = m_dot_bb - m_dot_cc

    dm_g = dm_g + m_dot_bbcc
    dm_l = dm_l - m_dot_bbcc
    dT_l = dT_l + (-m_dot_bb * (h_g - h_l) + P * m_dot_bbcc / rho_l - m_dot_bbcc * (h_l - u_l)) / (A_thermal - B_coupling)
    dT_g = dT_g + (m_dot_cc * (h_g - h_l) - P * m_dot_bbcc / rho_l + m_dot_bbcc * (h_g - u_g)) / (m_g_c * cv_g)
    dV_l = dm_l / rho_l - (m_l_c / rho_l**2) * drho_l_dT_l * dT_l
    dV_g = -dV_l
    m_dot_bo_final = m_dot_bo_final + m_dot_bbcc

    # Smooth restoring term outside the property table's range (for hydrogen the upper edge is the critical
    # temperature), so the integrator cannot drift beyond the tabulated states
    T_barrier_rate = 0.05  # 1/s, restoring-rate strength per K of excess
    T_eps          = 0.1   # K, smoothing width
    dT_g = dT_g + T_barrier_rate * soft_temperature_barrier(np.atleast_1d(T_g), T_g_lo, T_g_hi, T_eps)
    dT_l = dT_l + T_barrier_rate * soft_temperature_barrier(np.atleast_1d(T_l), T_l_lo, T_l_hi, T_eps)

    # Observability, not correction: exported as-is so a run relying on the
    # floors/barriers above to stay numerically alive is visible in the output.
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
                        m_dot_deficit=m_dot_deficit, m_dot_avail=m_dot_avail,
                        heater_saturated=heater_saturated)
    return derivatives, diagnostics


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
def compute_liquid_height(r, l, v_array):
    """Vectorized: solves for liquid height given an array of liquid volumes."""
    v_array  = np.clip(np.atleast_1d(np.asarray(v_array, dtype=float)), 0.0, None)
    v_full   = np.pi * r**2 * l + (4 / 3) * np.pi * r**3
    v_clipped = np.clip(v_array, 1e-9, v_full - 1e-9)
    h_array = np.fromiter(
        (brentq(liquid_height_residual, 1e-9, 2 * r - 1e-9, args=(r, l, vv)) for vv in v_clipped),
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
