# VnV/Verification/powertrain/cryogenic_tank_performance_test.py
#
#
# Created: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units, Data
from RCAIDE.Framework.Mission.Common import State, Conditions
from RCAIDE.Library.Methods.Utilities.Chebyshev.chebyshev_data import chebyshev_data
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank.compute_rounded_end_cylindrical_tank_volume import compute_rounded_end_cylindrical_tank_volume
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_cylindrical_tank_volume import compute_cryogenic_cylindrical_tank_volume
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.append_cryogenic_tank_conditions import append_cryogenic_tank_conditions
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_tank_performance import compute_cryogenic_tank_performance

import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Test harness: builds a single Cryogenic_Tank and calls its performance function
#  directly at a fixed cruise altitude and fixed engine fuel demand -- isolated from
#  the full mission solver, aerodynamic surrogates, and other tank instances, so
#  failures here are unambiguously in the tank physics rather than aircraft-level
#  coupling. compute_cryogenic_tank_performance solves the tank's own 6-state
#  boil-off IVP internally (adaptive Radau) and populates tank_conditions directly --
#  there is no outer residual/unknown machinery left to drive here.
# ----------------------------------------------------------------------------------------------------------------------
def build_tank(fuel, design_inlet_temperature, diameter, length):
    tank                            = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank()
    tank.tag                        = 'cryo_tank'
    tank.fuel                       = fuel
    tank.design_inlet_temperature   = design_inlet_temperature
    tank.design_altitude            = 35000. * Units.ft
    tank.design_heat_flux           = 20.0
    tank.design_total_heat_transfer = 2000.0
    tank.ullage_volume_fraction     = 0.07
    tank.diameters.external         = diameter
    tank.lengths.external           = length
    tank.xz_plane_symmetric         = False
    tank.assigned_distributors      = [['fuel_line']]
    tank.power_split_ratio          = 1.0

    compute_rounded_end_cylindrical_tank_volume(tank)
    compute_cryogenic_cylindrical_tank_volume(tank, Data(cryo_tank=tank))
    tank.fuel.mass_properties.mass = tank.volume_properties.net_volume * tank.fuel.density

    return tank


def build_state(n_nodes, duration, chemical_power_demand, cruise_altitude):
    state       = State()
    state._size = n_nodes

    x, D, I = chebyshev_data(n_nodes, integration=True)
    state.numerics.time.differentiate = D / duration
    state.numerics.time.integrate      = I * duration
    state.numerics.time.control_points = x * duration

    state.conditions.weights                  = Conditions()
    state.conditions.weights.components       = Conditions()
    state.conditions.weights.components.mass  = Conditions()

    state.conditions.energy               = Conditions()
    state.conditions.energy.sources       = Conditions()
    state.conditions.energy.distributors  = Conditions()
    state.conditions.energy.distributors['fuel_line'] = Conditions()
    state.conditions.energy.distributors['fuel_line'].outputs = Conditions()
    state.conditions.energy.distributors['fuel_line'].outputs.power = Conditions()
    state.conditions.energy.distributors['fuel_line'].outputs.power.chemical = chemical_power_demand * state.ones_row(1)

    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data  = atmosphere.compute_values(cruise_altitude)
    state.conditions.freestream = Conditions()
    state.conditions.freestream.temperature           = float(np.ravel(atmo_data.temperature)[0])           * state.ones_row(1)
    state.conditions.freestream.kinematic_viscosity    = float(np.ravel(atmo_data.kinematic_viscosity)[0])    * state.ones_row(1)
    state.conditions.freestream.prandtl_number          = float(np.ravel(atmo_data.prandtl_number)[0])          * state.ones_row(1)
    state.conditions.freestream.thermal_conductivity    = float(np.ravel(atmo_data.thermal_conductivity)[0])    * state.ones_row(1)

    return state


def build_case(fuel, design_inlet_temperature, chemical_power_demand, n_nodes, diameter, length, duration_hr):
    duration = duration_hr * 3600.0
    tank     = build_tank(fuel, design_inlet_temperature, diameter=diameter, length=length)
    state    = build_state(n_nodes, duration, chemical_power_demand, cruise_altitude=35000. * Units.ft)

    segment       = Data()
    segment.state = state
    append_cryogenic_tank_conditions(tank, segment)

    return tank, state


def run_case(fuel, design_inlet_temperature, chemical_power_demand, label, n_nodes=8,
             diameter=2.0, length=6.0, duration_hr=2.0, check_truth=False):
    tank, state = build_case(fuel, design_inlet_temperature, chemical_power_demand,
                              n_nodes, diameter, length, duration_hr)

    compute_cryogenic_tank_performance(tank, state, network=None)

    tank_conditions = state.conditions.energy.sources[tank.tag]
    m_g = tank_conditions.ullage_mass[:,0]
    m_l = tank_conditions.fuel_mass[:,0]
    total_mass_0 = m_g[0] + m_l[0]
    total_mass_f = m_g[-1] + m_l[-1]

    m_dot_l_engine       = chemical_power_demand / fuel.lower_heating_value
    m_dot_total_out      = m_dot_l_engine + tank_conditions.vent_rate[:,0]
    I                    = state.numerics.time.integrate
    expected_mass_loss   = np.dot(I, m_dot_total_out)[-1]
    actual_mass_loss     = total_mass_0 - total_mass_f

    print(f"\n----- {label} -----")
    print(f"  Initial total mass (liquid+ullage): {total_mass_0:.4f} kg")
    print(f"  Final total mass (liquid+ullage):   {total_mass_f:.4f} kg")
    print(f"  Expected mass loss (engine + vent): {expected_mass_loss:.4f} kg")
    print(f"  Actual mass loss:                    {actual_mass_loss:.4f} kg")
    print(f"  Boil-off rate range: [{tank_conditions.boil_off_flow_rate[:,0].min():.3e}, {tank_conditions.boil_off_flow_rate[:,0].max():.3e}] kg/s")
    print(f"  Heater power range:  [{tank_conditions.heater_power[:,0].min():.3e}, {tank_conditions.heater_power[:,0].max():.3e}] W")
    print(f"  Pressure range:      [{tank_conditions.pressure[:,0].min():.3e}, {tank_conditions.pressure[:,0].max():.3e}] Pa")

    mass_error = abs(actual_mass_loss - expected_mass_loss)
    assert mass_error < 1e-3, f"{label}: mass conservation violated by {mass_error:.3e} kg"
    assert np.all(m_g > 0) and np.all(m_l > 0), f"{label}: non-physical negative mass in solution"

    (T_l_lo, T_l_hi), _ = fuel.property_table_range(phase='liquid')
    T_l_clamped  = np.clip(tank_conditions.fuel_temperature[:,0], T_l_lo, T_l_hi)
    rho_l_check  = fuel.cryogen_properties(T_l_clamped, "Density (kg/m3)", phase='liquid')
    V_l_expected = m_l / rho_l_check
    vol_drift    = np.abs(tank_conditions.fuel_volume[:,0] - V_l_expected).max()
    print(f"  Max fuel_volume drift from m_l/rho_l(T_l): {vol_drift:.3e} m^3")
    assert vol_drift < 1e-6, f"{label}: fuel_volume drifted from m_l/rho_l(T_l) by {vol_drift:.3e} m^3"

    assert not np.any(tank_conditions.temperature_out_of_range[:,0]), (
        f"{label}: temperature_out_of_range tripped unexpectedly in a nominal case"
    )

    if check_truth:
        # Independent cross-check: re-solve the identical problem at a much tighter
        # IVP tolerance (fresh tank/state, since compute_cryogenic_tank_performance
        # writes its result into tank_conditions in place) and compare trajectories.
        # Explicitly pinned to Radau (rather than leaving method at its own default)
        # so this is a genuinely independent cross-check -- a different integrator
        # AND a much tighter tolerance than the production default (LSODA/1e-4/1e-7,
        # chosen for speed -- see compute_cryogenic_tank_performance's docstring),
        # not just the same method re-run tighter. This verifies the production
        # default is actually converged, not just that the integrator reports
        # success -- the same role the old collocation-vs-IVP cross-check served
        # before the tank was decoupled from the mission solver's Newton unknowns.
        tank_tight, state_tight = build_case(fuel, design_inlet_temperature, chemical_power_demand,
                                              n_nodes, diameter, length, duration_hr)
        compute_cryogenic_tank_performance(tank_tight, state_tight, network=None,
                                            rtol=1e-9, atol=1e-12, method='Radau')
        tc_tight = state_tight.conditions.energy.sources[tank_tight.tag]

        keys = ('ullage_mass', 'fuel_mass', 'ullage_temperature', 'fuel_temperature', 'ullage_volume', 'fuel_volume')
        x_sol   = np.concatenate([tank_conditions[k][:,0] for k in keys])
        x_truth = np.concatenate([tc_tight[k][:,0]        for k in keys])
        rel_errors    = np.abs(x_sol - x_truth) / (np.abs(x_truth) + 1e-6)
        max_rel_error = rel_errors.max()
        print(f"  Max relative error vs. tight-tolerance IVP truth trajectory: {max_rel_error:.3e}")
        assert max_rel_error < 1e-2, f"{label}: default-tolerance solution disagrees with tight-tolerance truth trajectory by {max_rel_error:.3e} (relative)"

    return tank, state


def refuel_cutoff_test():
    print('\n----- Refuel event cutoff -----')

    lh2 = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    tank, state = build_case(lh2, design_inlet_temperature=20.0, chemical_power_demand=0.0,
                              n_nodes=10, diameter=2.0, length=6.0, duration_hr=2.0)
    tank_conditions = state.conditions.energy.sources[tank.tag]

    full_mass   = tank.design_full_liquid_mass
    m_l_current = 0.2 * full_mass
    tank_conditions.fuel_mass[:, 0] = m_l_current

    T_g0  = tank_conditions.ullage_temperature[0, 0]
    rho_l0 = tank.fuel.cryogen_properties(tank_conditions.fuel_temperature[0, 0],
                                           "Density (kg/m3)", phase='liquid')
    V_l0  = m_l_current / rho_l0
    V_g0  = tank.volume_properties.gross_volume - V_l0
    P_sat0 = tank.fuel.cryogen_properties(T_g0, "Pressure (MPa)", phase='vapor') * 1e6
    R_specific = 8314.462618 / tank.fuel.molecular_weight
    tank_conditions.ullage_mass[:, 0] = P_sat0 * V_g0 / (R_specific * T_g0)

    target_mass = full_mass
    duration    = np.ravel(state.numerics.time.control_points)[-1] - np.ravel(state.numerics.time.control_points)[0]
    naive_rate  = max(0.0, target_mass - m_l_current) / duration
    tank_conditions.refuel_mass_flow_rate[:, 0] = 1.2 * naive_rate
    tank_conditions.refuel_target_mass[:, 0]    = target_mass

    compute_cryogenic_tank_performance(tank, state, network=None)

    m_l             = tank_conditions.fuel_mass[:, 0]
    V_l             = tank_conditions.fuel_volume[:, 0]
    refuel_rate     = tank_conditions.refuel_mass_flow_rate[:, 0]
    volume_capped   = bool(tank_conditions.refuel_volume_capped[0, 0])
    net_volume      = tank.volume_properties.net_volume
    heater_saturated = tank_conditions.heater_saturated[:, 0]

    rel_error = abs(m_l[-1] - target_mass) / target_mass
    print(f"  Final fuel_mass: {m_l[-1]:.4f} kg vs target {target_mass:.4f} kg (rel. error {rel_error:.2e})")
    print(f"  Final fuel_volume: {V_l[-1]:.4f} m^3 vs net_volume {net_volume:.4f} m^3")
    print(f"  volume-limited cutoff: {volume_capped}")
    print(f"  refuel_mass_flow_rate at segment end: {refuel_rate[-1]:.6e} kg/s")
    print(f"  heater saturated at any point: {bool(np.any(heater_saturated))}")

    (T_l_lo_r, T_l_hi_r), _ = tank.fuel.property_table_range(phase='liquid')
    T_l_clamped_r = np.clip(tank_conditions.fuel_temperature[:, 0], T_l_lo_r, T_l_hi_r)
    rho_l_check_r = tank.fuel.cryogen_properties(T_l_clamped_r, "Density (kg/m3)", phase='liquid')
    m_l_clamped_r = np.clip(m_l, 1e-6, None)
    vol_drift_r   = np.abs(V_l - m_l_clamped_r / rho_l_check_r).max()
    assert vol_drift_r < 1e-6, f"fuel_volume drifted from m_l/rho_l(T_l) by {vol_drift_r:.3e} m^3"
    assert np.any(heater_saturated), (
        "Expected the pressure-regulation heater to saturate during this fast, "
        "low-pressure-start refuel"
    )

    if volume_capped:
        assert V_l.max() <= net_volume * (1 + 1e-3), f"Refuel overshot net_volume, max V_l = {V_l.max():.4f} m^3"
    else:
        assert rel_error < 1e-3, f"Refuel should end at target_mass, rel. error {rel_error:.2e}"
        assert m_l.max() <= target_mass * (1 + 1e-3), f"Refuel overshot target_mass, max = {m_l.max():.4f} kg"
    assert refuel_rate[-1] == 0.0, f"refuel_mass_flow_rate should cut off to zero once done, got {refuel_rate[-1]:.6e} kg/s"
    assert np.any(refuel_rate == 0.0) and np.any(refuel_rate > 0.0), \
        "Expected a fill phase followed by a zero-rate hold phase, not a single constant rate"

    tank2, state2 = build_case(lh2, design_inlet_temperature=20.0, chemical_power_demand=0.0,
                                n_nodes=10, diameter=2.0, length=6.0, duration_hr=2.0)
    tc2 = state2.conditions.energy.sources[tank2.tag]
    tc2.refuel_mass_flow_rate[:, 0] = 1.2 * naive_rate
    tc2.refuel_target_mass[:, 0]    = tc2.fuel_mass[0, 0]

    compute_cryogenic_tank_performance(tank2, state2, network=None)
    assert np.all(tc2.refuel_mass_flow_rate[:, 0] == 0.0), (
        "A tank already at/above target shouldn't have any refuel flow applied, got "
        f"{tc2.refuel_mass_flow_rate[:, 0]}"
    )

    print('  PASSED')
    return


def near_empty_dormancy_test():
    print('\n----- Near-empty tank, long dormancy -----')

    lh2 = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    tank, state = build_case(lh2, design_inlet_temperature=20.0, chemical_power_demand=0.0,
                              n_nodes=16, diameter=2.6, length=4.5, duration_hr=5.0)
    tank_conditions = state.conditions.energy.sources[tank.tag]

    almost_empty = 0.10 * tank.design_full_liquid_mass
    tank_conditions.fuel_mass[:, 0]   = almost_empty

    compute_cryogenic_tank_performance(tank, state, network=None)

    m_l = tank_conditions.fuel_mass[:, 0]
    pressure = tank_conditions.pressure[:, 0]
    floor_active = tank_conditions.liquid_thermal_floor_active[:, 0]
    gate         = tank_conditions.liquid_availability_gate[:, 0]
    print(f"  fuel_mass range: [{m_l.min():.4f}, {m_l.max():.4f}] kg")
    print(f"  pressure range:  [{pressure.min():.3e}, {pressure.max():.3e}] Pa")
    print(f"  liquid_thermal_floor_active any: {bool(np.any(floor_active))}")
    print(f"  liquid_availability_gate min: {gate.min():.4f}")

    assert m_l.min() >= 0.0, f"fuel_mass went negative: min = {m_l.min():.4f} kg"
    assert pressure.max() < 5 * tank.design_pressure, (
        f"pressure ran away: max = {pressure.max():.3e} Pa vs design {tank.design_pressure:.3e} Pa"
    )
    assert np.any(floor_active) or gate.min() < 0.999, (
        "Expected the near-empty floor/gate mechanisms to engage at some point "
        "in this near-empty dormancy test -- if they never activate, this test "
        "isn't actually exercising what it claims to"
    )

    print('  PASSED')
    return


def main():
    lh2 = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    run_case(lh2, design_inlet_temperature=20.0, chemical_power_demand=2.0e5, label="LH2 cryogenic tank")

    lng = RCAIDE.Library.Attributes.Propellants.Liquid_Natural_Gas()
    run_case(lng, design_inlet_temperature=112.0, chemical_power_demand=2.0e5, label="LNG cryogenic tank")

    # Larger tank, near-dormancy draw, 1-hour window -- roughly matching the scale
    # of the paper's own validation case (Section V.C: 14.68 m^3 liquid + 0.815 m^3
    # ullage, 20 W total heat ingress, 25K/20K initial ullage/liquid temps, 1 hour,
    # validated internally against a conventional IVP solver).
    run_case(lh2, design_inlet_temperature=20.0, chemical_power_demand=1.0e3,
              label="LH2 large tank, near-dormancy", diameter=2.6, length=4.5, duration_hr=1.0,
              check_truth=True)

    refuel_cutoff_test()
    near_empty_dormancy_test()

    print("\nAll cryogenic tank performance tests passed.")


if __name__ == '__main__':
    main()
