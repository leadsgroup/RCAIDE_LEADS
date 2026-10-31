# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/Cryogenic_Tank/append_cryogenic_tank_conditions.py
#
# Created: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.append_fuel_tank_conditions import append_fuel_tank_conditions
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.compute_fuel_tank_capacity  import compute_fuel_tank_capacity
from RCAIDE.Framework.Core.Physical_Constants import UNIVERSAL_GAS_CONSTANT
from RCAIDE.Framework.Core import Units
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------
def append_cryogenic_tank_conditions(tank, segment):
    """
    Appends the conditions of a cryogenic tank: the base fuel tank conditions plus the two-phase states
    (ullage and liquid mass and temperature), volumes, pressure and the diagnostics of
    compute_cryogenic_tank_performance.

    The initial liquid mass is tank.fuel.mass_properties.mass when set, capped at the full liquid mass
    (net volume times liquid density at design_inlet_temperature), and the full liquid mass otherwise. The
    mass_properties Pre_Process sets the fuel load and appends these conditions again. The liquid starts at
    design_inlet_temperature and the ullage as saturated vapor at minimum_operating_pressure (P_sat when unset).
    Later segments start from the previous segment's end state (append_cryogenic_tank_segment_conditions).
    """
    append_fuel_tank_conditions(tank, segment)

    ones_row = segment.state.ones_row
    tank_conditions = segment.state.conditions.energy.sources[tank.tag]

    T_l_0 = tank.design_inlet_temperature
    P_sat = tank.fuel.cryogen_properties(T_l_0, "Pressure (MPa)", phase='liquid') * Units.MPa
    P_0   = P_sat if tank.minimum_operating_pressure is None else max(P_sat, tank.minimum_operating_pressure)
    T_g_0 = tank.fuel.saturation_temperature(np.atleast_1d(P_0 / Units.MPa))[0] if P_0 > P_sat else T_l_0

    rho_l_0         = tank.fuel.cryogen_properties(T_l_0, "Density (kg/m3)", phase='liquid')
    max_liquid_mass = tank.design_full_liquid_mass

    # Only the first segment can honor a partial-fill request -- later segments get
    # overwritten by append_cryogenic_tank_segment_conditions's continuity shift anyway.
    is_first_segment = not segment.state.initials.keys()

    if is_first_segment and tank.fuel.mass_properties.mass != 0:
        m_l_0 = tank.fuel.mass_properties.mass
        if m_l_0 > max_liquid_mass * (1 + 1e-6):
            print(f"Warning: {tank.tag} requested fuel mass ({m_l_0:.1f} kg) exceeds "
                  f"this tank's capacity ({max_liquid_mass:.1f} kg); clamping to capacity.")
            m_l_0 = max_liquid_mass
            tank.fuel.mass_properties.mass = m_l_0  # keep mass_properties Pre_Process in sync
        V_l_0 = m_l_0 / rho_l_0
    else:
        m_l_0 = max_liquid_mass
        V_l_0 = tank.volume_properties.net_volume

    V_g_0 = tank.volume_properties.gross_volume - V_l_0

    R_specific = UNIVERSAL_GAS_CONSTANT / tank.fuel.molecular_weight
    Z_0        = tank.fuel.compressibility_factor(T_g_0, phase='vapor')
    m_g_0      = P_0 * V_g_0 / (Z_0 * R_specific * T_g_0)

    tank_conditions.ullage_mass             = m_g_0 * ones_row(1)
    tank_conditions.fuel_mass               = m_l_0 * ones_row(1)
    tank_conditions.ullage_temperature      = T_g_0 * ones_row(1)
    tank_conditions.fuel_temperature        = T_l_0 * ones_row(1)
    tank_conditions.ullage_volume           = V_g_0 * ones_row(1)
    tank_conditions.fuel_volume             = V_l_0 * ones_row(1)
    tank_conditions.pressure                = 0 * ones_row(1)
    tank_conditions.vent_rate               = 0 * ones_row(1)
    tank_conditions.refuel_return_rate      = 0 * ones_row(1)
    tank_conditions.heater_power            = 0 * ones_row(1)
    tank_conditions.refuel_mass_flow_rate   = 0 * ones_row(1)
    tank_conditions.environmental_heat_leak_liquid = 0 * ones_row(1)
    tank_conditions.environmental_heat_leak_ullage = 0 * ones_row(1)
    tank_conditions.temperature_out_of_range    = 0 * ones_row(1)
    tank_conditions.liquid_thermal_floor_active = 0 * ones_row(1)
    tank_conditions.fuel_deficit_rate           = 0 * ones_row(1)
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
    Shifts the tank states (ullage and liquid mass and temperature) so node 0 matches the previous segment's
    end state. This runs on every iteration, so the states are shifted rather than overwritten; once node 0
    matches, the shift does nothing. Volumes are algebraic and are recomputed by
    compute_cryogenic_tank_performance.
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
