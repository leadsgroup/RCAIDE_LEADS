# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/Cryogenic_Tank/cryogenic_tank_rates_kernel.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
try:
    from numba import njit
except ImportError:  # numba is optional: without it the kernel runs as plain Python
    def njit(*args, **kwargs):
        if len(args) == 1 and callable(args[0]):
            return args[0]
        return lambda function: function

from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_tank_heat_leak import (
    CYLINDER, CUBOID, brent_heat_leak, heat_balance_residual)

# Column order of the property arrays built by cryogenic_tank_rates_tables
LIQUID_COLUMNS = ("Density (kg/m3)", "Cp (J/g*K)", "Viscosity (Pa*s)", "Therm. Cond. (W/m*K)", "Enthalpy (kJ/kg)",
                  "Internal Energy (kJ/kg)", "Pressure (MPa)")
VAPOR_COLUMNS  = ("Density (kg/m3)", "Cp (J/g*K)", "Cv (J/g*K)", "Viscosity (Pa*s)", "Therm. Cond. (W/m*K)",
                  "Enthalpy (kJ/kg)", "Internal Energy (kJ/kg)", "Pressure (MPa)")
# Order of the diagnostics returned by cryogenic_tank_rates (as tank_state_rates' diagnostics dict)
DIAGNOSTICS    = ("P", "m_dot_vent", "m_dot_bo_final", "heater_power", "m_dot_l_engine", "Q_e_l", "Q_e_g", "V_l", "V_g",
                  "T_out_of_range", "m_l_thermal_floored", "m_barrier_blend", "m_dot_avail", "heater_saturated")
MPA            = 1e6
EPS4           = 4 * np.finfo(float).eps
DELTA_T        = 0.1  # K, interface temperature-difference smoothing (as Q_liq_to_int/Q_gas_to_int)


# ----------------------------------------------------------------------------------------------------------------------
#  Tables and parameters (built once per tank solve)
# ----------------------------------------------------------------------------------------------------------------------
def cryogenic_tank_rates_tables(tank, R_specific, T_g_lo, T_g_hi, T_l_lo, T_l_hi, P_lo, P_hi, max_heater_power):
    """
    Packs everything cryogenic_tank_rates needs into plain arrays: the fuel's saturated property
    tables (the same rows its interpolators use) and the tank's scalar parameters and geometry.

    Returns
    -------
    (TL, LQ, TV, VQ, PS, TS, params, geometry)
    """
    fuel   = tank.fuel
    liquid = fuel.saturation_table('liquid')
    vapor  = fuel.saturation_table('vapor')
    TL     = liquid["Temperature (K)"]
    LQ     = np.ascontiguousarray(np.vstack([liquid[name] for name in LIQUID_COLUMNS]))
    TV     = vapor["Temperature (K)"]
    VQ     = np.ascontiguousarray(np.vstack([vapor[name] for name in VAPOR_COLUMNS]))
    PS     = liquid["Pressure (MPa)"]

    s = tank.inner_structure
    if tank.geometry_type == 'cylindrical':
        kind     = CYLINDER
        geometry = np.array([s.diameters.internal / 2, s.lengths.internal, s.diameters.internal / 2, 0.,
                             s.diameters.external / 2, s.diameters.internal / 2, s.lengths.internal, 0.], dtype=float)
    else:
        kind     = CUBOID
        geometry = np.array([s.lengths.internal, s.widths.internal, s.heights.internal, 0.,
                             s.lengths.external, s.widths.external, s.heights.external, s.thickness], dtype=float)
    params = np.array([kind, 2 if tank.xz_plane_symmetric else 1, tank.volume_properties.gross_volume, R_specific,
                       fuel.lower_heating_value, T_g_lo, T_g_hi, T_l_lo, T_l_hi, P_lo, P_hi, max_heater_power,
                       tank.insulation.thickness, tank.inner_structure.material.thermal_conductivity,
                       tank.insulation.material.thermal_conductivity, tank.design_full_liquid_mass,
                       tank.pressure_regulation_time_constant, tank.design_pressure,
                       tank.heater_direct_boiloff_fraction], dtype=float)
    return TL, LQ, TV, VQ, PS, liquid["Temperature (K)"], params, geometry


# ----------------------------------------------------------------------------------------------------------------------
#  Scalar kernels
# ----------------------------------------------------------------------------------------------------------------------
@njit(cache=True)
def liquid_height_residual(h, r, l, v):
    return l * (r**2 * np.arccos((r - h) / r) - (r - h) * np.sqrt(2 * r * h - h**2)) + (np.pi / 3) * h**2 * (3 * r - h) - v


@njit(cache=True)
def liquid_height(r, l, v):
    """Liquid height in a rounded-end cylinder: scipy brentq (xtol 2e-12, rtol 4 eps) on
    liquid_height_residual, as compute_liquid_height."""
    v_full = np.pi * r**2 * l + (4 / 3) * np.pi * r**3
    v      = min(max(max(v, 0.0), 1e-9), v_full - 1e-9)
    xpre, xcur = 1e-9, 2 * r - 1e-9
    xblk, fblk, spre, scur = 0., 0., 0., 0.
    fpre = liquid_height_residual(xpre, r, l, v)
    fcur = liquid_height_residual(xcur, r, l, v)
    if fpre == 0:
        return xpre
    if fcur == 0:
        return xcur
    for i in range(100):
        if fpre != 0 and fcur != 0 and np.signbit(fpre) != np.signbit(fcur):
            xblk = xpre
            fblk = fpre
            spre = scur = xcur - xpre
        if abs(fblk) < abs(fcur):
            xpre = xcur
            xcur = xblk
            xblk = xpre
            fpre = fcur
            fcur = fblk
            fblk = fpre
        delta = (2e-12 + EPS4 * abs(xcur)) / 2
        sbis  = (xblk - xcur) / 2
        if fcur == 0 or abs(sbis) < delta:
            return xcur
        if abs(spre) > delta and abs(fcur) < abs(fpre):
            if xpre == xblk:
                stry = -fcur * (xcur - xpre) / (fcur - fpre)
            else:
                dpre = (fpre - fcur) / (xpre - xcur)
                dblk = (fblk - fcur) / (xblk - xcur)
                stry = -fcur * (fblk * dblk - fpre * dpre) / (dblk * dpre * (fblk - fpre))
            if 2 * abs(stry) < min(abs(spre), 3 * abs(sbis) - delta):
                spre = scur
                scur = stry
            else:
                spre = sbis
                scur = sbis
        else:
            spre = sbis
            scur = sbis
        xpre = xcur
        fpre = fcur
        if abs(scur) > delta:
            xcur += scur
        else:
            xcur += delta if sbis > 0 else -delta
        fcur = liquid_height_residual(xcur, r, l, v)
    return xcur


@njit(cache=True)
def environment_heat_leak(kind, t_ins, T_env, T_cold, k_mat, k_ins, k_air, nu, alpha, Pr, geometry):
    """Heat leak through wall + insulation (solve_heat_leak). Returns (Q, status): status 0 ok, else the
    Python solver's fallback/raise path is needed."""
    if kind == CYLINDER:
        p = np.array([t_ins, T_env, T_cold, k_mat, k_ins, k_air, nu, alpha, Pr, geometry[4], geometry[5], geometry[6]])
    else:
        p = np.array([t_ins, T_env, T_cold, k_mat, k_ins, k_air, nu, alpha, Pr, geometry[4], geometry[5], geometry[6], geometry[7]])
    if abs(T_env - T_cold) < 1e-9:
        Te = T_cold
    else:
        lo = min(T_cold, T_env)
        hi = max(T_cold, T_env)
        if not (np.isfinite(lo) and np.isfinite(hi)):
            return 0.0, 1
        Te, status = brent_heat_leak(kind, lo, hi, 1e-9, EPS4, 100, p)
        if status != 0:
            return 0.0, status
    return heat_balance_residual(kind, Te, p)[1], 0


@njit(cache=True)
def interface_heat(T_bulk, T_int, A_int, L_int, rho, cp, mu, k):
    """Natural-convection heat from a bulk region to the interface (Q_liq_to_int / Q_gas_to_int)."""
    dT    = T_bulk - T_int
    GrPr  = (L_int**3 * rho**2 * 9.81 * (1.0 / T_bulk) * np.sqrt(dT**2 + DELTA_T**2) * cp) / (mu * k + 1e-12)
    alpha = 0.27 * (k / L_int) * (GrPr**0.25)
    return alpha * A_int * dT


@njit(cache=True)
def soft_temperature_barrier(T, T_lo, T_hi, eps):
    above = -0.5 * ((T - T_hi) + np.sqrt((T - T_hi)**2 + eps**2))
    below = 0.5 * ((T_lo - T) + np.sqrt((T_lo - T)**2 + eps**2))
    return above + below


@njit(cache=True)
def cryogenic_tank_rates(t_i, y, t, inputs, TL, LQ, TV, VQ, PS, TS, params, geometry):
    """
    Integrator right-hand side of compute_cryogenic_tank_performance's 4-state tank model (the same
    equations as tank_state_rates, scalar and compiled). inputs rows: chemical power, T_env, nu_air,
    Pr_air, k_air, refuel rate, at the segment's control points t.

    Returns
    -------
    (rates, diagnostics, status): rates = [dm_g, dm_l, dT_g, dT_l]; diagnostics in DIAGNOSTICS order;
    status != 0 means a heat-leak solve needs the Python path, and neither may be used.
    """
    rates       = np.zeros(4)
    diagnostics = np.zeros(len(DIAGNOSTICS))
    m_g, m_l, T_g, T_l = y[0], y[1], y[2], y[3]
    chemical_power = np.interp(t_i, t, inputs[0])
    T_env          = np.interp(t_i, t, inputs[1])
    nu_air         = np.interp(t_i, t, inputs[2])
    Pr_air         = np.interp(t_i, t, inputs[3])
    k_air          = np.interp(t_i, t, inputs[4])
    m_dot_refuel   = np.interp(t_i, t, inputs[5])

    kind, n_sides, gross_volume, R_specific, LHV = int(params[0]), params[1], params[2], params[3], params[4]
    T_g_lo, T_g_hi, T_l_lo, T_l_hi, P_lo, P_hi   = params[5], params[6], params[7], params[8], params[9], params[10]
    max_heater_power, t_ins, k_mat, k_ins        = params[11], params[12], params[13], params[14]
    m_full, tau, design_pressure, eta_h          = params[15], params[16], params[17], params[18]

    m_g_c = max(m_g, 1e-6)
    m_l_c = max(m_l, 1e-6)
    T_g_c = min(max(T_g, T_g_lo), T_g_hi)
    T_l_c = min(max(T_l, T_l_lo), T_l_hi)

    rho_l   = np.interp(T_l_c, TL, LQ[0])
    cp_liq  = np.interp(T_l_c, TL, LQ[1]) * 1000.0
    mu_liq  = np.interp(T_l_c, TL, LQ[2])
    k_liq   = np.interp(T_l_c, TL, LQ[3])
    h_l     = np.interp(T_l_c, TL, LQ[4]) * 1000.0
    u_l     = np.interp(T_l_c, TL, LQ[5]) * 1000.0
    P_sat_l = np.interp(T_l_c, TL, LQ[6]) * MPA
    V_l_c   = m_l_c / rho_l
    V_g_c   = max(gross_volume - V_l_c, 1e-6)

    T_plus      = min(max(T_l_c + 0.01, T_l_lo), T_l_hi)
    T_minus     = min(max(T_l_c - 0.01, T_l_lo), T_l_hi)
    dT_span     = max(T_plus - T_minus, 1e-6)
    drho_l_dT_l = (np.interp(T_plus, TL, LQ[0]) - np.interp(T_minus, TL, LQ[0])) / dT_span
    dPsat_dT_l  = (np.interp(T_plus, TL, LQ[6]) - np.interp(T_minus, TL, LQ[6])) * MPA / dT_span

    m_dot_l_engine = chemical_power / LHV

    Z         = np.interp(T_g_c, TV, VQ[7]) * MPA / (np.interp(T_g_c, TV, VQ[0]) * R_specific * T_g_c)
    P         = Z * (m_g_c / V_g_c) * R_specific * T_g_c
    P_clamped = min(max(P / MPA, P_lo), P_hi)
    T_int     = np.interp(P_clamped, PS, TS)

    v_side = V_l_c / n_sides
    if kind == CYLINDER:
        r_in, l_in = geometry[0], geometry[1]
        h          = liquid_height(r_in, l_in, v_side)
        lamda      = h / (2 * r_in)
        L_int      = 4 * r_in * np.sqrt(lamda - lamda**2)
        A_int      = (np.pi * 2 * r_in * L_int**2 / 4) + L_int * l_in
        A_wet_frac = np.arccos(min(max((r_in - h) / r_in, -1.0), 1.0)) / np.pi
    else:
        l_in, w_in, H_in = geometry[0], geometry[1], geometry[2]
        v_box      = min(max(v_side, 0.0), l_in * w_in * H_in)
        h          = v_box / (l_in * w_in)
        L_int      = np.sqrt(l_in * w_in)
        A_int      = l_in * w_in
        A_wet_frac = min(max(h / H_in, 0.0), 1.0)
    A_int = A_int * n_sides

    rho_g = np.interp(T_g_c, TV, VQ[0])
    cp_g  = np.interp(T_g_c, TV, VQ[1]) * 1000.0
    cv_g  = np.interp(T_g_c, TV, VQ[2]) * 1000.0
    mu_g  = np.interp(T_g_c, TV, VQ[3])
    k_g   = np.interp(T_g_c, TV, VQ[4])
    h_g   = np.interp(T_g_c, TV, VQ[5]) * 1000.0
    u_g   = np.interp(T_g_c, TV, VQ[6]) * 1000.0

    Q_l_i = interface_heat(T_l_c, T_int, A_int, L_int, rho_l, cp_liq, mu_liq, k_liq)
    Q_g_i = interface_heat(T_g_c, T_int, A_int, L_int, rho_g, cp_g, mu_g, k_g)

    alpha_air = nu_air / Pr_air
    Q_env_liq, status = environment_heat_leak(kind, t_ins, T_env, T_l_c, k_mat, k_ins, k_air, nu_air, alpha_air, Pr_air, geometry)
    if status != 0:
        return rates, diagnostics, status
    Q_env_gas, status = environment_heat_leak(kind, t_ins, T_env, T_g_c, k_mat, k_ins, k_air, nu_air, alpha_air, Pr_air, geometry)
    if status != 0:
        return rates, diagnostics, status
    Q_e_l = n_sides * Q_env_liq * A_wet_frac
    Q_e_g = n_sides * Q_env_gas * (1 - A_wet_frac)

    m_dot_bo    = (Q_l_i + Q_g_i) / (h_g - h_l)
    m_l_floor   = 1e-3 * m_full
    m_avail_eps = 5e-4 * m_full
    m_dot_avail = 0.5 * (1.0 + (m_l - m_l_floor) / np.sqrt((m_l - m_l_floor)**2 + m_avail_eps**2))
    m_dot_bo    = m_dot_bo * m_dot_avail

    m_dot_reg        = (V_g_c / (R_specific * T_g_c * tau)) * (design_pressure - P)
    eps              = 1e-6
    m_dot_heater_boi = 0.5 * ( m_dot_reg + np.sqrt(m_dot_reg**2 + eps**2)) * m_dot_avail
    m_dot_vent       = 0.5 * (-m_dot_reg + np.sqrt(m_dot_reg**2 + eps**2))

    heater_power_uncapped = m_dot_heater_boi * (h_g - h_l) / max(eta_h, 1e-9)
    heater_power          = min(heater_power_uncapped, max_heater_power)
    m_dot_heater_boi      = heater_power * eta_h / (h_g - h_l)
    m_dot_bo_final        = m_dot_bo + m_dot_heater_boi
    Q_h_to_liquid         = heater_power * (1 - eta_h)

    dm_g = m_dot_bo_final - m_dot_vent
    dm_l = -m_dot_bo_final - m_dot_l_engine + m_dot_refuel

    m_barrier_blend = min(max((m_l_floor + m_avail_eps - m_l) / (2 * m_avail_eps), 0.0), 1.0)
    m_barrier_blend = m_barrier_blend**2 * (3 - 2 * m_barrier_blend)
    dm_l = dm_l + m_barrier_blend * (m_l_floor - m_l) / tau

    m_l_thermal = max(m_l_c, m_l_floor)
    A_thermal   = m_l_thermal * cp_liq
    B_coupling  = P * m_l_c / rho_l**2 * drho_l_dT_l
    RHS0        = -Q_l_i + Q_e_l + Q_h_to_liquid - P * dm_l / rho_l + dm_l * (h_l - u_l)
    dT_l        = RHS0 / (A_thermal - B_coupling)
    dV_l        = dm_l / rho_l - (m_l_c / rho_l**2) * drho_l_dT_l * dT_l
    dV_g        = -dV_l
    dT_g        = (-Q_g_i + Q_e_g - P * dV_g + dm_g * (h_g - u_g)) / (m_g_c * cv_g)

    # Bulk boiling and cloud condensation, Adler & Martins (2025) Eqs. 36-59
    P_dot      = P * (dm_g / m_g_c + dT_g / T_g_c - dV_g / V_g_c)
    dP_fd      = 1e-3 * P_clamped
    dTsat_dP   = (np.interp(min(max(P_clamped + dP_fd, P_lo), P_hi), PS, TS) -
                  np.interp(min(max(P_clamped - dP_fd, P_lo), P_hi), PS, TS)) / (2 * dP_fd * MPA)
    m_dot_bb   = abs(V_g_c / (Z * R_specific * T_g_c) * (dPsat_dT_l * dT_l - P_dot))
    m_dot_cc   = abs(P * V_g_c / (Z * R_specific * T_g_c**2) * (dTsat_dP * P_dot - dT_g))
    s_bb       = 1.0 / (1.0 + np.exp(min(max(100.0 * (P - P_sat_l) / P_sat_l, -50.0), 50.0)))
    s_cc       = 1.0 / (1.0 + np.exp(min(max(100.0 * (T_g_c - T_int) / T_int, -50.0), 50.0)))
    m_dot_bb   = s_bb * m_dot_bb * m_dot_avail
    m_dot_cc   = s_cc * m_dot_cc
    m_dot_bbcc = m_dot_bb - m_dot_cc

    dm_g = dm_g + m_dot_bbcc
    dm_l = dm_l - m_dot_bbcc
    dT_l = dT_l + (-m_dot_bb * (h_g - h_l) + P * m_dot_bbcc / rho_l - m_dot_bbcc * (h_l - u_l)) / (A_thermal - B_coupling)
    dT_g = dT_g + (m_dot_cc * (h_g - h_l) - P * m_dot_bbcc / rho_l + m_dot_bbcc * (h_g - u_g)) / (m_g_c * cv_g)

    dT_g = dT_g + 0.05 * soft_temperature_barrier(T_g, T_g_lo, T_g_hi, 0.1)
    dT_l = dT_l + 0.05 * soft_temperature_barrier(T_l, T_l_lo, T_l_hi, 0.1)

    rates[0] = dm_g
    rates[1] = dm_l
    rates[2] = dT_g
    rates[3] = dT_l
    diagnostics[0]  = P
    diagnostics[1]  = m_dot_vent
    diagnostics[2]  = m_dot_bo_final + m_dot_bbcc
    diagnostics[3]  = heater_power
    diagnostics[4]  = m_dot_l_engine
    diagnostics[5]  = Q_e_l
    diagnostics[6]  = Q_e_g
    diagnostics[7]  = V_l_c
    diagnostics[8]  = V_g_c
    diagnostics[9]  = 1.0 if (T_g < T_g_lo or T_g > T_g_hi or T_l < T_l_lo or T_l > T_l_hi) else 0.0
    diagnostics[10] = 1.0 if m_l_c < m_l_floor else 0.0
    diagnostics[11] = m_barrier_blend
    diagnostics[12] = m_dot_avail
    diagnostics[13] = 1.0 if heater_power_uncapped > max_heater_power else 0.0
    return rates, diagnostics, 0


def cryogenic_tank_node_diagnostics(t, m_g, m_l, T_g, T_l, inputs, tables):
    """
    Diagnostics at each control point (the states and inputs there), as tank_state_rates' dict:
    name -> array over nodes. Returns None if any node needs the Python heat-leak path.
    """
    values = np.zeros((len(DIAGNOSTICS), len(t)))
    for k in range(len(t)):
        _, values[:, k], status = cryogenic_tank_rates(t[k], np.array([m_g[k], m_l[k], T_g[k], T_l[k]]), t, inputs, *tables)
        if status != 0:
            return None
    diagnostics = {name: values[i] for i, name in enumerate(DIAGNOSTICS)}
    for name in ("T_out_of_range", "m_l_thermal_floored", "heater_saturated"):
        diagnostics[name] = diagnostics[name] > 0.5
    return diagnostics
