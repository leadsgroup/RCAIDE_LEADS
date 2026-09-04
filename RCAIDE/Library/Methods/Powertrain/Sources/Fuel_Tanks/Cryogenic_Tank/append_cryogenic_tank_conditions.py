# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/Cryogenic_Tank/append_cryogenic_tank_conditions.py
#
# Created: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.append_fuel_tank_conditions import append_fuel_tank_conditions
from RCAIDE.Framework.Core.Physical_Constants import UNIVERSAL_GAS_CONSTANT
from RCAIDE.Framework.Core import Units
import numpy as np

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

    Initial ullage mass is set from the saturation pressure at
    ``design_inlet_temperature`` (``P_sat(T)``, via the same real-gas EOS used by
    ``compute_cryogenic_tank_performance``), NOT from ``tank.design_pressure``
    (``= P_sat(T) + pressure_margin``). Liquid and ullage both start at the same
    temperature, and a liquid/vapor pair at a single temperature has exactly one
    physically achievable equilibrium pressure -- P_sat(T). Seeding at
    design_pressure instead (strictly above P_sat) would describe vapor that's
    already over-pressurized relative to its own liquid at that same T, which
    isn't a state the two phases could actually be found in at rest: real vapor
    that supersaturated in contact with its liquid condenses immediately, driving
    P back toward P_sat(T). design_pressure remains the runtime target
    m_dot_reg regulates the ullage toward once the mission starts (matching a
    real tank being pressurized up after fill) -- it's a regulation setpoint,
    not an achievable instantaneous phase-equilibrium state, so it has no
    business seeding node 0.

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

    R_specific = UNIVERSAL_GAS_CONSTANT / tank.fuel.molecular_weight
    Z_0        = tank.fuel.compressibility_factor(T_g_0, phase='vapor')
    P_sat_0    = tank.fuel.cryogen_properties(T_g_0, "Pressure (MPa)", phase='vapor') * Units.MPa  # Pa
    m_g_0      = P_sat_0 * V_g_0 / (Z_0 * R_specific * T_g_0)

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
    tank_conditions.environmental_heat_leak_liquid = 0 * ones_row(1)
    tank_conditions.environmental_heat_leak_ullage = 0 * ones_row(1)
    tank_conditions.temperature_out_of_range    = 0 * ones_row(1)
    tank_conditions.liquid_thermal_floor_active = 0 * ones_row(1)
    tank_conditions.liquid_mass_floor_blend     = 0 * ones_row(1)
    tank_conditions.liquid_availability_gate    = 1 * ones_row(1)
    tank_conditions.heater_saturated            = 0 * ones_row(1)
    tank_conditions.refuel_volume_capped        = 0 * ones_row(1)

    tank_conditions.refuel_target_mass      = np.nan * ones_row(1)

    segment.state.conditions.weights.components.mass[tank.fuel.tag] = m_l_0 * ones_row(1)

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

    ullage_volume/fuel_volume are NOT shifted here -- they're algebraic outputs
    of compute_cryogenic_tank_performance (V_l = m_l/rho_l(T_l)), not
    independent states, so they get recomputed correctly and consistently the
    moment that function runs for this segment; shifting a stale value here
    would just be overwritten anyway.
    """
    if not segment.state.initials.keys():
        return

    tank_conditions = segment.state.conditions.energy.sources[tank.tag]
    prior           = segment.state.initials.conditions.energy.sources[tank.tag]

    for field in ('ullage_mass', 'fuel_mass', 'ullage_temperature', 'fuel_temperature'):
        current = tank_conditions[field]
        target  = prior[field][-1,0]
        tank_conditions[field][:,:] = current + (target - current[0,0])

    return
