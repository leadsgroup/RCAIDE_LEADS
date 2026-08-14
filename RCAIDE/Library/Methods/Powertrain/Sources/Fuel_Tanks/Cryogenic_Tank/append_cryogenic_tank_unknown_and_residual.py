# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/Cryogenic_Tank/append_cryogenic_tank_unknown_and_residual.py
#
# Created: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------
def append_cryogenic_tank_unknown_and_residual(tank, segment):
    """
    Registers the 6 network unknowns/residuals for a cryogenic tank's in-flight
    boil-off model: ullage mass, liquid mass, ullage temperature, liquid temperature,
    ullage volume, liquid volume. Initial values are read from the tank conditions
    already populated by ``append_cryogenic_tank_conditions``.

    The ullage regulation flow (heater-driven boil-off if positive, vent if negative)
    is deliberately NOT a 7th unknown here: it is computed explicitly, inside
    ``compute_cryogenic_tank_performance``, from a finite-gain feedback law driven by
    the current pressure error (P - tank.design_pressure) and
    tank.pressure_regulation_time_constant, the same way m_dot_bo is already computed
    explicitly from interfacial heat transfer. An earlier version of this model
    enforced pressure exactly via an added 7th algebraic unknown/residual pair
    (P(m_g,T_g,V_g) = design_pressure at every node); that gives the regulation
    mechanism infinite gain/authority -- an idealization stiffer than the finite
    heater/vent authority either Adler & Martins (2025) or the AST paper actually
    describe -- and proved numerically fragile in the warm-start integration.
    """
    ones_row        = segment.state.ones_row
    tank_conditions = segment.state.conditions.energy.sources[tank.tag]

    segment.state.number_of_network_unknowns  += 6
    segment.state.number_of_network_residuals += 6

    # A perfectly flat initial guess is degenerate for the Chebyshev
    # differentiation matrix D used downstream (D @ constant = 0), which
    # collapses every non-pinned node's residual to the same single-point rate
    # and can stall the solver before it ever sees the mission's actual time
    # dependence. Nudge with a small monotonic ramp to break that degeneracy --
    # a coarse index-based ramp rather than a real rate estimate, since the
    # real time grid/differentiation matrix aren't populated yet at this point
    # in Pre_Process. Each state's shift is scaled off its OWN initial value,
    # not shared across states: ullage mass/volume can be orders of magnitude
    # smaller than liquid mass/volume (e.g. an ullage sized against
    # design_pressure rather than saturated vapor density), so sizing the
    # ullage shift off the liquid scale can be a large fraction of the (much
    # smaller) ullage value -- confirmed to blow up the very first residual
    # evaluation (pressure spiking node-to-node in the raw initial guess) when
    # tried.
    n_nodes = ones_row(1).shape[0]
    ramp    = np.linspace(0.0, 1.0, n_nodes).reshape(-1, 1) if n_nodes > 1 else ones_row(1) * 0.0

    m_g_0 = tank_conditions.ullage_mass[0,0]
    m_l_0 = tank_conditions.fuel_mass[0,0]
    V_g_0 = tank_conditions.ullage_volume[0,0]
    V_l_0 = tank_conditions.fuel_volume[0,0]
    ullage_mass_shift   = 1e-3 * m_g_0 * ramp
    fuel_mass_shift     = 1e-3 * m_l_0 * ramp
    ullage_volume_shift = 1e-3 * V_g_0 * ramp
    fuel_volume_shift   = 1e-3 * V_l_0 * ramp

    segment.state.unknowns.network[tank.tag + '_ullage_mass']        = ones_row(1) * m_g_0 + ullage_mass_shift
    segment.state.unknowns.network[tank.tag + '_fuel_mass']          = ones_row(1) * m_l_0 - fuel_mass_shift
    segment.state.unknowns.network[tank.tag + '_ullage_temperature'] = ones_row(1) * tank_conditions.ullage_temperature[0,0]
    segment.state.unknowns.network[tank.tag + '_fuel_temperature']   = ones_row(1) * tank_conditions.fuel_temperature[0,0]
    segment.state.unknowns.network[tank.tag + '_ullage_volume']      = ones_row(1) * V_g_0 + ullage_volume_shift
    segment.state.unknowns.network[tank.tag + '_fuel_volume']        = ones_row(1) * V_l_0 - fuel_volume_shift

    segment.state.residuals.network[tank.tag + '_ullage_mass']        = ones_row(1) * 0
    segment.state.residuals.network[tank.tag + '_fuel_mass']          = ones_row(1) * 0
    segment.state.residuals.network[tank.tag + '_ullage_temperature'] = ones_row(1) * 0
    segment.state.residuals.network[tank.tag + '_fuel_temperature']   = ones_row(1) * 0
    segment.state.residuals.network[tank.tag + '_ullage_volume']      = ones_row(1) * 0
    segment.state.residuals.network[tank.tag + '_fuel_volume']        = ones_row(1) * 0

    V_g_0 = tank_conditions.ullage_volume[0,0]

    (T_l_lo, T_l_hi), _ = tank.fuel.property_table_range(phase='liquid')
    (T_g_lo, T_g_hi), _ = tank.fuel.property_table_range(phase='vapor')

    segment.state.unknowns_lower_bounds.network[tank.tag + '_ullage_mass']        = 1e-6 * ones_row(1)
    segment.state.unknowns_upper_bounds.network[tank.tag + '_ullage_mass']        = m_l_0 * ones_row(1)
    segment.state.unknowns_lower_bounds.network[tank.tag + '_fuel_mass']          = 1e-6 * ones_row(1)
    segment.state.unknowns_upper_bounds.network[tank.tag + '_fuel_mass']          = m_l_0 * ones_row(1)
    segment.state.unknowns_lower_bounds.network[tank.tag + '_ullage_temperature'] = T_g_lo * ones_row(1)
    segment.state.unknowns_upper_bounds.network[tank.tag + '_ullage_temperature'] = T_g_hi * ones_row(1)
    segment.state.unknowns_lower_bounds.network[tank.tag + '_fuel_temperature']   = T_l_lo * ones_row(1)
    segment.state.unknowns_upper_bounds.network[tank.tag + '_fuel_temperature']   = T_l_hi * ones_row(1)
    segment.state.unknowns_lower_bounds.network[tank.tag + '_ullage_volume']      = 1e-6 * ones_row(1)
    segment.state.unknowns_upper_bounds.network[tank.tag + '_ullage_volume']      = (V_l_0 + V_g_0) * ones_row(1)
    segment.state.unknowns_lower_bounds.network[tank.tag + '_fuel_volume']        = 1e-6 * ones_row(1)
    segment.state.unknowns_upper_bounds.network[tank.tag + '_fuel_volume']        = (V_l_0 + V_g_0) * ones_row(1)

    return
