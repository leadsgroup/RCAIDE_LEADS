"""Regression checks for fuel-load resolution, tank capacity, and MTOW limits."""

# VnV/Verification/powertrain/test_fuel_load_rules.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import importlib
from types import SimpleNamespace

import RCAIDE
from RCAIDE.Framework.Mission.Common                            import Conditions, State
from RCAIDE.Library.Mission.Common.Pre_Process.mass_properties   import *
from RCAIDE.Library.Methods.Geometry.Planform                    import compute_fuel_volume
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.compute_fuel_mass import   compute_fuel_mass
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.compute_fuel_tank_capacity import    compute_fuel_tank_capacity
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.compute_fuel_tank_performance import compute_fuel_tank_performance 
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.set_vehicle_fuel_mass import set_vehicle_fuel_mass 
import numpy as np

mass_properties_module = importlib.import_module(
    "RCAIDE.Library.Mission.Common.Pre_Process.mass_properties"
)


def main():
    check_compute_fuel_mass_respects_user_tank_load_precedence()
    check_compute_fuel_mass_distributes_vehicle_load_or_fills_capacity()
    check_set_vehicle_fuel_mass_uses_capacity_proportions_and_rejects_overfill()
    check_set_vehicle_fuel_mass_handles_mixed_conventional_and_cryogenic_tanks()
    check_max_fuel_iteration_sums_mixed_tank_capacities()
    check_computed_fuel_is_capped_by_mtow()
    check_computed_fuel_is_capped_by_tank_capacity()
    check_prescribed_masses_are_kept_and_flagged()
    check_oew_plus_payload_above_mtow_is_flagged()
    check_re_evaluation_clears_only_unchanged_computed_fuel_and_takeoff()
    check_tanks_keep_fixed_flow_shares_and_report_local_deficit()

    return 


# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def make_vehicle(tank_capacities, initial_masses=None):
    """Build a fuel network with Jet-A tanks sized to the requested capacities."""
    vehicle = RCAIDE.Vehicle()
    network = RCAIDE.Framework.Networks.Fuel()
    vehicle.networks.append(network)
    initial_masses = initial_masses or [0.0] * len(tank_capacities)

    tanks = []
    for index, (capacity, initial_mass) in enumerate(zip(tank_capacities, initial_masses)):
        tank = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank()
        tank.tag = f"tank_{index}"
        tank.fuel = RCAIDE.Library.Attributes.Propellants.Jet_A()
        tank.volume_properties.net_volume = capacity / tank.fuel.density
        tank.fuel.mass_properties.mass = initial_mass
        tank.assigned_distributors = [[network.tag]]
        network.sources.append(tank)
        tanks.append(tank)

    return vehicle, tanks


def assert_value_error(function, message):
    try:
        function()
    except ValueError as error:
        assert message in str(error), str(error)
    else:
        raise AssertionError(f"Expected ValueError containing: {message}")


# Explicit tank loads take precedence over the aggregate vehicle fuel value.
def check_compute_fuel_mass_respects_user_tank_load_precedence():
    vehicle, tanks = make_vehicle([100.0, 300.0], [15.0, 25.0])
    vehicle.mass_properties.fuel = 0.0

    compute_fuel_mass(vehicle)

    assert np.isclose(vehicle.mass_properties.fuel, 40.0)
    np.testing.assert_allclose(
        [tank.fuel.mass_properties.mass for tank in tanks], [15.0, 25.0]
    )


# Aggregate fuel is capacity-weighted across tanks, or defaults to full capacity.
def check_compute_fuel_mass_distributes_vehicle_load_or_fills_capacity():
    vehicle, tanks = make_vehicle([100.0, 300.0])
    vehicle.mass_properties.fuel = 80.0

    compute_fuel_mass(vehicle)

    assert np.isclose(vehicle.mass_properties.fuel, 80.0)
    np.testing.assert_allclose(
        [tank.fuel.mass_properties.mass for tank in tanks], [20.0, 60.0]
    )

    for tank in tanks:
        tank.fuel.mass_properties.mass = 0.0
    vehicle.mass_properties.fuel = 0.0
    compute_fuel_mass(vehicle)

    assert np.isclose(vehicle.mass_properties.fuel, 400.0)
    np.testing.assert_allclose(
        [tank.fuel.mass_properties.mass for tank in tanks], [100.0, 300.0]
    )


# Explicit vehicle fuel loads distribute by capacity and reject combined overfill.
def check_set_vehicle_fuel_mass_uses_capacity_proportions_and_rejects_overfill():
    vehicle, tanks = make_vehicle([100.0, 300.0])

    set_vehicle_fuel_mass(vehicle, 80.0)

    assert np.isclose(vehicle.mass_properties.fuel, 80.0)
    np.testing.assert_allclose(
        [tank.fuel.mass_properties.mass for tank in tanks], [20.0, 60.0]
    )

    assert_value_error(
        lambda: set_vehicle_fuel_mass(vehicle, 401.0), "combined capacity"
    )


# Conventional and cryogenic capacities both contribute to the assigned fuel load.
def check_set_vehicle_fuel_mass_handles_mixed_conventional_and_cryogenic_tanks():
    vehicle, tanks = make_vehicle([100.0])
    cryogenic_tank = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank()
    cryogenic_tank.tag = "lh2_tank"
    cryogenic_tank.fuel = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    cryogenic_tank.design_inlet_temperature = 20.0
    cryogenic_tank.volume_properties.net_volume = 2.0
    vehicle.networks.fuel.sources.append(cryogenic_tank)

    expected_capacities = [
        compute_fuel_tank_capacity(tanks[0]),
        compute_fuel_tank_capacity(cryogenic_tank),
    ]
    requested_fuel = 0.5 * sum(expected_capacities)
    set_vehicle_fuel_mass(vehicle, requested_fuel)

    expected_masses = [0.5 * capacity for capacity in expected_capacities]
    assert np.allclose(
        [tanks[0].fuel.mass_properties.mass, cryogenic_tank.fuel.mass_properties.mass],
        expected_masses,
    )
    assert np.isclose(vehicle.mass_properties.fuel, requested_fuel)

    compute_fuel_mass(vehicle, update_fuel_mass=False, update_max_fuel_mass=True)
    assert np.isclose(vehicle.mass_properties.max_fuel, sum(expected_capacities))


# Maximum fuel iteration sums conventional and cryogenic tank capacity.
def check_max_fuel_iteration_sums_mixed_tank_capacities():
    vehicle, tanks = make_vehicle([100.0])
    cryogenic_tank = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank()
    cryogenic_tank.tag = "lh2_tank"
    cryogenic_tank.fuel = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    cryogenic_tank.design_inlet_temperature = 20.0
    cryogenic_tank.volume_properties.net_volume = 2.0
    vehicle.networks.fuel.sources.append(cryogenic_tank)
    vehicle.mass_properties.max_takeoff = 10000.0
    vehicle.mass_properties.operating_empty = 1000.0
    vehicle.mass_properties.payload = 100.0
    vehicle.mass_properties.min_payload = 100.0
    vehicle.mass_properties.max_fuel = None
    vehicle.mass_properties.max_zero_fuel = None

    weights = SimpleNamespace(
        settings=SimpleNamespace(overwrite_operating_empty_weight=False),
        evaluate=lambda _: None,
    )
    analyses = SimpleNamespace(vehicle=vehicle, weights=weights)
    original_correction_factors = mass_properties_module.apply_correction_factors
    original_component_weights = mass_properties_module.apply_component_weights
    try:
        mass_properties_module.apply_correction_factors = lambda _: None
        mass_properties_module.apply_component_weights = lambda _: None
        mass_properties_module.iterate_max_fuel_and_max_zero_fuel(analyses)
    finally:
        mass_properties_module.apply_correction_factors = original_correction_factors
        mass_properties_module.apply_component_weights = original_component_weights

    expected_capacity = sum(compute_fuel_tank_capacity(tank) for tank in [*tanks, cryogenic_tank])
    assert np.isclose(vehicle.mass_properties.max_fuel, expected_capacity)


# Computed fuel fills the tanks up to MTOW and is split across the tanks by capacity.
def check_computed_fuel_is_capped_by_mtow():
    vehicle, tanks = make_vehicle([100.0, 300.0])
    vehicle.mass_properties.max_takeoff = 250.0
    vehicle.mass_properties.operating_empty = 150.0
    vehicle.mass_properties.payload = 50.0

    compute_mission_fuel_load(vehicle)

    assert np.isclose(vehicle.mass_properties.fuel, 50.0)
    np.testing.assert_allclose(
        [tank.fuel.mass_properties.mass for tank in tanks], [12.5, 37.5]
    )
    assert check_mass_limits(vehicle) == []


# Computed fuel for a prescribed takeoff mass is capped by tank capacity; the takeoff mass is kept and flagged.
def check_computed_fuel_is_capped_by_tank_capacity():
    vehicle, tanks = make_vehicle([60.0, 140.0])
    vehicle.mass_properties.max_takeoff = 600.0
    vehicle.mass_properties.operating_empty = 100.0
    vehicle.mass_properties.payload = 50.0
    vehicle.mass_properties.takeoff = 500.0

    compute_mission_fuel_load(vehicle, prescribed_takeoff=500.0)

    assert np.isclose(vehicle.mass_properties.fuel, 200.0)
    assert np.isclose(vehicle.mass_properties.takeoff, 500.0)
    np.testing.assert_allclose(
        [tank.fuel.mass_properties.mass for tank in tanks], [60.0, 140.0]
    )
    messages = check_mass_limits(vehicle, prescribed_takeoff=500.0)
    assert len(messages) == 1 and "differs from OEW plus payload plus fuel" in messages[0], messages


# Prescribed fuel and takeoff masses beyond the limits are kept as given and only flagged.
def check_prescribed_masses_are_kept_and_flagged():
    vehicle, tanks = make_vehicle([100.0, 300.0], [25.0, 75.0])
    vehicle.mass_properties.fuel = 100.0
    vehicle.mass_properties.max_takeoff = 250.0
    vehicle.mass_properties.operating_empty = 150.0
    vehicle.mass_properties.payload = 50.0
    messages = check_mass_limits(vehicle)
    assert len(messages) == 1 and "plus fuel (300.0 kg) exceeds MTOW" in messages[0], messages
    assert np.isclose(vehicle.mass_properties.fuel, 100.0)
    np.testing.assert_allclose(
        [tank.fuel.mass_properties.mass for tank in tanks], [25.0, 75.0]
    )

    messages = check_mass_limits(vehicle, prescribed_takeoff=300.0)
    assert any("exceeds MTOW (250.0 kg)" in message and "takeoff" in message for message in messages), messages

    messages = check_mass_limits(vehicle, prescribed_takeoff=180.0)
    assert any("is below OEW plus payload (200.0 kg)" in message for message in messages), messages


# OEW plus payload above MTOW is flagged and leaves no computed fuel.
def check_oew_plus_payload_above_mtow_is_flagged():
    vehicle, _ = make_vehicle([100.0])
    vehicle.mass_properties.max_takeoff = 500.0
    vehicle.mass_properties.operating_empty = 400.0
    vehicle.mass_properties.payload = 150.0

    compute_mission_fuel_load(vehicle)

    assert np.isclose(vehicle.mass_properties.fuel, 0.0)
    messages = check_mass_limits(vehicle)
    assert len(messages) == 1 and "OEW plus payload (550.0 kg) exceeds MTOW" in messages[0], messages

    return


# Re-evaluation clears computed values but preserves user-edited values.
def check_re_evaluation_clears_only_unchanged_computed_fuel_and_takeoff():
    vehicle, tanks = make_vehicle([100.0, 300.0], [20.0, 60.0])
    vehicle.mass_properties.fuel = 80.0
    vehicle.mass_properties.computed_fuel = 80.0
    vehicle.mass_properties.takeoff = 230.0
    vehicle.mass_properties.computed_takeoff = 230.0

    assert not clear_computed_fuel_and_takeoff(vehicle)
    assert np.isclose(vehicle.mass_properties.fuel, 0.0)
    assert vehicle.mass_properties.takeoff is None
    np.testing.assert_allclose(
        [tank.fuel.mass_properties.mass for tank in tanks], [0.0, 0.0]
    )

    vehicle.mass_properties.fuel = 45.0
    vehicle.mass_properties.takeoff = 220.0
    for tank, mass in zip(tanks, (10.0, 35.0)):
        tank.fuel.mass_properties.mass = mass
    assert clear_computed_fuel_and_takeoff(vehicle)
    assert np.isclose(vehicle.mass_properties.fuel, 45.0)
    assert np.isclose(vehicle.mass_properties.takeoff, 220.0)

    return 


# Fixed tank flow shares remain fixed and an exhausted tank reports its deficit.
def check_tanks_keep_fixed_flow_shares_and_report_local_deficit():
    vehicle, tanks = make_vehicle([100.0, 300.0], [0.5, 10.0])
    compute_fuel_volume(vehicle, compute_fuel_volume=False)
    assert np.allclose([tank.power_split_ratio for tank in tanks], [0.25, 0.75])

    state = State()
    state._size = 2
    state.numerics.number_of_control_points = 2
    state.numerics.time.integrate = np.array([[0.0, 0.0], [1.0, 0.0]])
    state.conditions.energy = Conditions()
    state.conditions.energy.sources = Conditions()
    state.conditions.energy.distributors = Conditions()
    distributor = Conditions()
    distributor.outputs = Conditions()
    distributor.outputs.power = Conditions()
    fuel = tanks[0].fuel
    total_fuel_flow = 4.0
    total_chemical_power = total_fuel_flow * fuel.lower_heating_value
    distributor.outputs.power.chemical = np.full((2, 1), total_chemical_power)
    state.conditions.energy.distributors["fuel"] = distributor

    for tank, initial_mass in zip(tanks, (0.5, 10.0)):
        share = tank.power_split_ratio
        tank_conditions = Conditions()
        tank_conditions.power_split_ratio = np.full((2, 1), share)
        tank_conditions.fuel_mass = np.full((2, 1), initial_mass)
        tank_conditions.boil_off_flow_rate = np.zeros((2, 1))
        tank_conditions.secondary_mass_flow_rate = np.zeros((2, 1))
        tank_conditions.refuel_mass_flow_rate = np.zeros((2, 1))
        tank_conditions.refuel_target_mass = np.full((2, 1), np.nan)
        tank_conditions.fuel_deficit_rate = np.zeros((2, 1))
        tank_conditions.inputs = Conditions()
        tank_conditions.outputs = Conditions()
        tank_conditions.outputs.power = Conditions()
        state.conditions.energy.sources[tank.tag] = tank_conditions

    for tank in tanks:
        compute_fuel_tank_performance(tank, state, distributor=None)

    small_tank = state.conditions.energy.sources[tanks[0].tag]
    large_tank = state.conditions.energy.sources[tanks[1].tag]
    large_tank_demand = total_fuel_flow * tanks[1].power_split_ratio
    expected_large_tank_mass = 10.0 - large_tank_demand
    assert np.isclose(small_tank.fuel_mass[-1, 0], 0.0)
    assert small_tank.fuel_deficit_rate[-1, 0] > 0.0
    assert np.isclose(large_tank.fuel_mass[-1, 0], expected_large_tank_mass)
    assert np.isclose(large_tank.fuel_deficit_rate[-1, 0], 0.0)

    return 

if __name__ == "__main__":
    main()