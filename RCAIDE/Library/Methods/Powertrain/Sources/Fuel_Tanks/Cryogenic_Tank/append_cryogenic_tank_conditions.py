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
    (``tank.volume_properties.net_volume`` is the liquid fuel volume, ullage excluded).
    The ullage is initialized at the same temperature as the liquid.

    Initial liquid mass is computed directly from ``net_volume x liquid density at
    design_inlet_temperature`` -- NOT read from ``tank.fuel.mass_properties.mass`` --
    for the same reason ullage mass is computed from the real-gas EOS below rather
    than from a stored attribute: this method runs during the ``energy`` Pre_Process
    step (``RCAIDE.Library.Mission.Common.Pre_Process.energy``), which executes
    *before* the ``mass_properties`` step
    (``RCAIDE.Library.Mission.Common.Pre_Process.mass_properties``) that calls
    ``compute_fuel_mass`` to populate ``tank.fuel.mass_properties.mass``. Reading that
    attribute here would silently pick up whatever stale/zero value it happened to
    hold before sizing -- confirmed directly: it read back as exactly 0 kg for two of
    the Hydrogen_BWB's three active tanks, which then integrated negative from the
    very first engine offtake. Deriving m_l_0 the same way rho_l is derived everywhere
    else in this model (``fuel.cryogen_properties(..., phase='liquid')``) makes it
    correct regardless of Pre_Process ordering, exactly like m_g_0 below.

    Initial ullage mass is set from ``tank.design_pressure`` via the same real-gas
    EOS used by ``compute_cryogenic_tank_performance`` (rather than from saturated
    vapor density) so the initial state already satisfies the runtime model's
    constant-pressure regulation constraint -- otherwise node 0 is over-determined
    (T_g, V_g, and m_g would all be independently pinned to values that generally
    don't satisfy P(m_g,T_g,V_g) = design_pressure exactly).

    This runs once, mission-wide, before any segment has actually converged, so it
    always sets the design-basis full tank -- cross-segment continuity is handled
    separately by append_cryogenic_tank_segment_conditions (called per-segment, when
    state.initials is actually populated).
    """
    append_fuel_tank_conditions(tank, segment)

    ones_row = segment.state.ones_row
    tank_conditions = segment.state.conditions.energy.sources[tank.tag]

    T_l_0 = tank.design_inlet_temperature
    T_g_0 = tank.design_inlet_temperature
    V_l_0 = tank.volume_properties.net_volume
    V_g_0 = tank.volume_properties.gross_volume - tank.volume_properties.net_volume

    rho_l_0 = tank.fuel.cryogen_properties(T_l_0, "Density (kg/m3)", phase='liquid')
    m_l_0   = V_l_0 * rho_l_0

    # Fixed design-basis full-tank mass (at design_inlet_temperature), independent of
    # whatever temperature the tank is actually at later -- e.g. Ground.Refuel targets
    # a fraction of this, not a fraction recomputed from a drifted current temperature.
    tank.design_full_liquid_mass = m_l_0

    R_specific = R_UNIVERSAL / tank.fuel.molecular_weight
    Z_0        = tank.fuel.compressibility_factor(T_g_0, phase='vapor')
    m_g_0      = tank.design_pressure * V_g_0 / (Z_0 * R_specific * T_g_0)

    tank_conditions.ullage_mass             = m_g_0 * ones_row(1)
    tank_conditions.fuel_mass               = m_l_0 * ones_row(1)
    tank_conditions.ullage_temperature      = T_g_0 * ones_row(1)
    tank_conditions.fuel_temperature        = T_l_0 * ones_row(1)
    tank_conditions.ullage_volume           = V_g_0 * ones_row(1)
    tank_conditions.fuel_volume             = V_l_0 * ones_row(1)
    tank_conditions.pressure                = 0 * ones_row(1)
    tank_conditions.vent_rate               = 0 * ones_row(1)
    tank_conditions.heater_power            = 0 * ones_row(1)
    tank_conditions.refuel_mass_flow_rate   = 0 * ones_row(1)

    return


# ----------------------------------------------------------------------------------------------------------------------
#  Per-segment continuity
# ----------------------------------------------------------------------------------------------------------------------
def append_cryogenic_tank_segment_conditions(tank, segment):
    """
    Anchors node 0 to the prior segment's end state, when chaining -- called
    via Cryogenic_Tank.append_segment_conditions (iterate.initials.energy),
    which runs after the prior segment has actually converged, unlike
    append_cryogenic_tank_conditions above (mission-wide, pre-solve).

    Shifts rather than overwrites (mirrors Common.Initialize.weights): this
    runs on every iterate, so a hard overwrite would erase the ODE's own
    already-solved trajectory on every call after the first. Shifting by
    (target - current[0,0]) is a no-op once node 0 already matches, since
    compute_cryogenic_tank_performance's own y0 read is what put it there.
    """
    if not segment.state.initials.keys():
        return

    tank_conditions = segment.state.conditions.energy.sources[tank.tag]
    prior           = segment.state.initials.conditions.energy.sources[tank.tag]

    for field in ('ullage_mass', 'fuel_mass', 'ullage_temperature',
                  'fuel_temperature', 'ullage_volume', 'fuel_volume'):
        current = tank_conditions[field]
        target  = prior[field][-1,0]
        tank_conditions[field][:,:] = current + (target - current[0,0])

    return
