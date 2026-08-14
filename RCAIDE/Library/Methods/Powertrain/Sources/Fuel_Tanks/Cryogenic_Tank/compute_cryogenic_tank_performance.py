# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/Cryogenic_Tank/compute_cryogenic_tank_performance.py
#
# Created: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
from scipy.optimize import brentq

from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_tank_heat_leak import compute_cryogenic_tank_heat_leak

R_UNIVERSAL = 8314.462618  # J/(kmol*K), i.e. J/(kg*K) per unit molecular weight in g/mol

# ----------------------------------------------------------------------------------------------------------------------
#  Cryogenic Tank In-Flight Boil-Off Performance
# ----------------------------------------------------------------------------------------------------------------------
def compute_cryogenic_tank_performance(tank, state, network):
    """
    Two-phase lumped-parameter in-flight boil-off model for a cryogenic fuel tank.

    Solves 6 coupled mass/volume/energy-balance residuals (ullage mass, liquid mass,
    ullage temperature, liquid temperature, ullage volume, liquid volume) via
    ``state.numerics.time.differentiate``, matching RCAIDE's existing pseudospectral
    collocation residual pattern. Fluid properties come from the saturated-property
    tables already bundled with the propellant classes
    (``tank.fuel.cryogen_properties`` / ``tank.fuel.saturation_temperature``); the
    ullage is treated as an ideal gas off the saturation dome.

    The instantaneous boil-off/vent split uses a smoothed complementarity relaxation
    (rather than a hard threshold) so the residual stays differentiable: when more
    boil-off is needed than passive heat leak alone provides, the excess is treated as
    active-heater duty (and its required power reported as an output, not fed back into
    the aircraft's electrical bus); when less is needed, the excess ullage vapor is
    vented.

    Notes
    -----
    Not modeled here (out of scope for this port): active-heater dynamics (treated as
    quasi-steady, no lag), bulk boiling / cloud condensation corrections, and pump
    power for active re-liquefaction.
    """
    D   = state.numerics.time.differentiate
    tag = tank.tag
    tank_conditions = state.conditions.energy.sources[tag]
    fuel = tank.fuel

    m_g = state.unknowns.network[tag + '_ullage_mass']
    m_l = state.unknowns.network[tag + '_fuel_mass']
    T_g = state.unknowns.network[tag + '_ullage_temperature']
    T_l = state.unknowns.network[tag + '_fuel_temperature']
    V_g = state.unknowns.network[tag + '_ullage_volume']
    V_l = state.unknowns.network[tag + '_fuel_volume']

    # ------------------------------------------------------------------
    #  Clamp iterates to physically valid ranges before evaluating any
    #  property lookup or EOS. This mission's root-finder solver ignores
    #  the registered unknown bounds during numerical Jacobian probing, so
    #  intermediate (non-converged) iterates can wander through zero/
    #  negative mass or outside the property table's range; the tables
    #  raise instead of extrapolating. Clamping keeps every evaluation
    #  finite so the solver can be pushed back into the feasible region
    #  instead of crashing. Residuals still difference against the raw
    #  (unclamped) unknown trajectory further down, so the converged
    #  solution itself is unaffected as long as it lies within bounds.
    # ------------------------------------------------------------------
    (T_l_lo, T_l_hi), (P_lo, P_hi) = fuel.property_table_range(phase='liquid')
    (T_g_lo, T_g_hi), _            = fuel.property_table_range(phase='vapor')

    m_g_c = np.clip(m_g[:,0], 1e-6, None)
    m_l_c = np.clip(m_l[:,0], 1e-6, None)
    V_g_c = np.clip(V_g[:,0], 1e-6, None)
    V_l_c = np.clip(V_l[:,0], 1e-6, None)
    T_g_c = np.clip(T_g[:,0], T_g_lo, T_g_hi)
    T_l_c = np.clip(T_l[:,0], T_l_lo, T_l_hi)

    # ------------------------------------------------------------------
    #  Engine fuel offtake demand (same derivation the base explicit-
    #  integration fuel tank model uses: distributor's already-computed
    #  chemical power demand, split by power_split_ratio)
    # ------------------------------------------------------------------
    chemical_power = tank_conditions.power_split_ratio[:,0] * \
        state.conditions.energy.distributors[tank.assigned_distributors[0][0]].outputs.power.chemical[:,0]
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
    R_specific = R_UNIVERSAL / fuel.molecular_weight   # J/(kg*K)
    Z         = fuel.compressibility_factor(T_g_c, phase='vapor')
    P         = Z * (m_g_c / V_g_c) * R_specific * T_g_c   # Pa
    P_clamped = np.clip(P / 1e6, P_lo, P_hi)                # table pressure column is in MPa
    T_int     = fuel.saturation_temperature(P_clamped)

    # ------------------------------------------------------------------
    #  Interface geometry from the current liquid volume
    # ------------------------------------------------------------------
    L_int, A_int, A_wet_frac = compute_interface_geometric_properties(tank, V_l_c)

    # ------------------------------------------------------------------
    #  Bulk liquid/vapor properties at the current bulk temperatures
    # ------------------------------------------------------------------
    rho_l  = fuel.cryogen_properties(T_l_c, "Density (kg/m3)",     phase='liquid')
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
    T_env     = state.conditions.freestream.temperature[:,0]
    nu_air    = state.conditions.freestream.kinematic_viscosity[:,0]
    Pr_air    = state.conditions.freestream.prandtl_number[:,0]
    k_air     = state.conditions.freestream.thermal_conductivity[:,0]
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
        # Conformal/prismatic tanks are sized with a simple 1D-conduction insulation
        # model (compute_cryogenic_conformal_tank_volume.py), not the Churchill outer
        # convection/radiation correlation -- mirrored here for consistency rather
        # than forcing a cylindrical model onto box geometry.
        l_i = tank.inner_structure.lengths.internal
        w_i = tank.inner_structure.widths.internal
        h_i = tank.inner_structure.heights.internal
        A_box = 2 * (l_i * w_i + l_i * h_i + w_i * h_i)
        Q_env_liq = k_ins_mat * A_box * (T_env - T_l_c) / t_ins
        Q_env_gas = k_ins_mat * A_box * (T_env - T_g_c) / t_ins

    Q_e_l = Q_env_liq * A_wet_frac
    Q_e_g = Q_env_gas * (1 - A_wet_frac)

    # ------------------------------------------------------------------
    #  Natural boil-off from interfacial heat transfer
    # ------------------------------------------------------------------
    m_dot_bo = (Q_l_i + Q_g_i) / (h_g - h_l)

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
    # m_dot_reg > 0 -> active-heater boil-off duty; m_dot_reg < 0 -> venting
    eps               = 1e-6  # kg/s, smoothing width
    m_dot_heater_boi  = 0.5 * (m_dot_reg + np.sqrt(m_dot_reg**2 + eps**2))
    m_dot_vent        = m_dot_heater_boi - m_dot_reg
    m_dot_bo_final    = m_dot_bo + m_dot_heater_boi

    # Heater power split, per Adler & Martins (2025) Eq. 13/17/24: only a
    # fraction eta_h of the heater's total power goes directly to flash-
    # boiling liquid at the interface (the m_dot_heater_boi duty above); the
    # remaining (1 - eta_h) fraction instead raises the bulk liquid
    # temperature (their Eq. 17 Q_h(1-eta_h) term) rather than boiling it
    # outright. m_dot_heater_boi*(h_g-h_l) is therefore only the eta_h-share
    # of the total heater power, not the total.
    eta_h         = tank.heater_direct_boiloff_fraction
    heater_power  = m_dot_heater_boi * (h_g - h_l) / eta_h   # total heater power (diagnostic; not on the electrical bus)
    Q_h_to_liquid = heater_power * (1 - eta_h)                # fraction warming the bulk liquid, not boiling it

    # ------------------------------------------------------------------
    #  Mass / volume / energy balances
    # ------------------------------------------------------------------
    dm_g = m_dot_bo_final - m_dot_vent
    dm_l = -m_dot_bo_final - m_dot_l_engine
    dV_g = -dm_l / rho_l
    dV_l = dm_l / rho_l

    dT_g = (-Q_g_i + Q_e_g - P * dV_g + dm_g * (h_g - u_g)) / (m_g_c * cv_g)
    dT_l = (-Q_l_i + Q_e_l + Q_h_to_liquid - P * dV_l + dm_l * (h_l - u_l)) / (m_l_c * cp_liq)

    # Soft restoring term outside the property table's range. The table's edges are
    # not arbitrary data cutoffs -- e.g. hydrogen's upper bound is its actual critical
    # temperature, above which there is no distinct saturated liquid/vapor at all -- so
    # clamping only the property *lookups* (above) leaves the residual itself flat
    # (insensitive) once T exceeds the table, and a solver can converge to a
    # self-consistent but unphysical "runaway" branch out there. This adds a real
    # restoring force so the dynamics themselves resist leaving the valid domain,
    # smoothed the same way as the boil-off/vent split above.
    T_barrier_rate = 0.05  # 1/s, restoring-rate strength per K of excess
    T_eps          = 0.1   # K, smoothing width
    dT_g += T_barrier_rate * _soft_temperature_barrier(T_g[:,0], T_g_lo, T_g_hi, T_eps)
    dT_l += T_barrier_rate * _soft_temperature_barrier(T_l[:,0], T_l_lo, T_l_hi, T_eps)

    # ------------------------------------------------------------------
    #  Residuals: D*y - dy/dt = 0 at every collocation node, pinned at
    #  index 0 to the segment's initial conditions
    # ------------------------------------------------------------------
    R_ullage_mass            = np.dot(D, m_g)[:,0] - dm_g
    R_ullage_mass[0]         = m_g[0,0] - tank_conditions.ullage_mass[0,0]

    R_fuel_mass               = np.dot(D, m_l)[:,0] - dm_l
    R_fuel_mass[0]            = m_l[0,0] - tank_conditions.fuel_mass[0,0]

    R_ullage_volume            = np.dot(D, V_g)[:,0] - dV_g
    R_ullage_volume[0]         = V_g[0,0] - tank_conditions.ullage_volume[0,0]

    R_fuel_volume               = np.dot(D, V_l)[:,0] - dV_l
    R_fuel_volume[0]            = V_l[0,0] - tank_conditions.fuel_volume[0,0]

    R_ullage_temperature       = np.dot(D, T_g)[:,0] - dT_g
    R_ullage_temperature[0]    = T_g[0,0] - tank_conditions.ullage_temperature[0,0]

    R_fuel_temperature          = np.dot(D, T_l)[:,0] - dT_l
    R_fuel_temperature[0]       = T_l[0,0] - tank_conditions.fuel_temperature[0,0]

    state.residuals.network[tag + '_ullage_mass'][:,0]        = R_ullage_mass
    state.residuals.network[tag + '_fuel_mass'][:,0]          = R_fuel_mass
    state.residuals.network[tag + '_ullage_volume'][:,0]      = R_ullage_volume
    state.residuals.network[tag + '_fuel_volume'][:,0]        = R_fuel_volume
    state.residuals.network[tag + '_ullage_temperature'][:,0] = R_ullage_temperature
    state.residuals.network[tag + '_fuel_temperature'][:,0]   = R_fuel_temperature

    # ------------------------------------------------------------------
    #  Store outputs
    # ------------------------------------------------------------------
    tank_conditions.ullage_mass[1:,0]        = m_g[1:,0]
    tank_conditions.fuel_mass[1:,0]          = m_l[1:,0]
    tank_conditions.ullage_volume[1:,0]      = V_g[1:,0]
    tank_conditions.fuel_volume[1:,0]        = V_l[1:,0]
    tank_conditions.ullage_temperature[1:,0] = T_g[1:,0]
    tank_conditions.fuel_temperature[1:,0]   = T_l[1:,0]
    tank_conditions.pressure[:,0]            = P
    tank_conditions.vent_rate[:,0]           = m_dot_vent
    tank_conditions.boil_off_flow_rate[:,0]  = m_dot_bo_final
    tank_conditions.heater_power[:,0]        = heater_power

    # Total mass leaving the tank system (engine offtake + vented boil-off) --
    # what drives vehicle weight/CG bookkeeping in Common/Update/weights.py
    tank_conditions.mass_flow_rate[:,0]         = m_dot_l_engine + m_dot_vent
    tank_conditions.outputs.power.chemical[:,0] = m_dot_l_engine * fuel.lower_heating_value

    stored_results_flag = True
    stored_source_tag    = tank.tag
    return tank_conditions.inputs, tank_conditions.outputs, stored_results_flag, stored_source_tag


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
