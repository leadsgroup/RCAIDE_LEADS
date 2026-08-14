# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/Cryogenic_Tank/append_cryogenic_tank_conditions.py
#
# Created: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.append_fuel_tank_conditions import append_fuel_tank_conditions

R_UNIVERSAL = 8314.462618  # J/(kmol*K)

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------
def append_cryogenic_tank_conditions(tank, segment):
    """
    Appends initial conditions for a cryogenic fuel tank's in-flight boil-off model,
    extending the base fuel tank conditions with the fields needed by the two-phase
    (liquid/ullage) mass-volume-energy balance.

    Initial liquid state comes from the tank's design-time sizing
    (``tank.volume_properties.net_volume`` is the liquid fuel volume, ullage excluded;
    ``tank.fuel.mass_properties.mass`` the corresponding liquid mass -- both already
    populated by ``compute_fuel_mass``/``compute_cryogenic_cylindrical_tank_volume``).
    The ullage is initialized at the same temperature as the liquid.

    Initial ullage mass is set from ``tank.design_pressure`` via the same real-gas
    EOS used by ``compute_cryogenic_tank_performance`` (rather than from saturated
    vapor density) so the initial state already satisfies the runtime model's
    constant-pressure regulation constraint -- otherwise node 0 is over-determined
    (T_g, V_g, and m_g would all be independently pinned to values that generally
    don't satisfy P(m_g,T_g,V_g) = design_pressure exactly).
    """
    append_fuel_tank_conditions(tank, segment)

    ones_row = segment.state.ones_row
    tank_conditions = segment.state.conditions.energy.sources[tank.tag]

    T_l_0 = tank.design_inlet_temperature
    T_g_0 = tank.design_inlet_temperature
    V_l_0 = tank.volume_properties.net_volume
    V_g_0 = tank.volume_properties.gross_volume - tank.volume_properties.net_volume
    m_l_0 = tank.fuel.mass_properties.mass

    R_specific = R_UNIVERSAL / tank.fuel.molecular_weight
    Z_0        = tank.fuel.compressibility_factor(T_g_0, phase='vapor')
    m_g_0      = tank.design_pressure * V_g_0 / (Z_0 * R_specific * T_g_0)

    tank_conditions.ullage_mass        = m_g_0 * ones_row(1)
    tank_conditions.fuel_mass          = m_l_0 * ones_row(1)
    tank_conditions.ullage_temperature = T_g_0 * ones_row(1)
    tank_conditions.fuel_temperature   = T_l_0 * ones_row(1)
    tank_conditions.ullage_volume      = V_g_0 * ones_row(1)
    tank_conditions.fuel_volume        = V_l_0 * ones_row(1)
    tank_conditions.pressure           = 0 * ones_row(1)
    tank_conditions.vent_rate          = 0 * ones_row(1)
    tank_conditions.heater_power       = 0 * ones_row(1)

    return
