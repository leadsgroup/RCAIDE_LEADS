# VnV/Verification/framework_core/test_configurations.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys
from copy import deepcopy

import RCAIDE
from RCAIDE.Framework.Core import Units, Data, configuration_values
from RCAIDE.Library.Mission.Common.Pre_Process import configurations, apply_configuration

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # A mission flies one vehicle. A configuration (flaps, landing gear, fan speed) is the set of values in which it
    # differs from that vehicle, written onto the vehicle before a segment is set up, solved or read. Checked on the
    # Boeing 737 and the mission of the turbofan network test, without solving it.
    vnv_folder   = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    start_folder = os.getcwd()
    sys.path.insert(0, os.path.join(vnv_folder, "Vehicles"))
    os.chdir(os.path.join(vnv_folder, "Verification", "network_turbofan"))
    sys.path.insert(0, os.getcwd())
    import turbofan_network_test as test

    vehicle = test.vehicle_setup()
    configs = test.configs_setup(vehicle)

    check_values_are_the_edits(vehicle, configs)
    check_applied_vehicle_equals_configuration(vehicle, configs)
    check_order_of_application(vehicle, configs)

    mission = test.missions_setup(test.mission_setup(test.analyses_setup(configs))).base_mission
    check_segments_share_one_vehicle(vehicle, mission)
    check_unknown_value_raises(test)

    os.chdir(start_folder)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_values_are_the_edits(vehicle, configs):
    # the landing configuration of the vehicle script: flaps 30 degrees, slats 25 degrees, landing gear down
    changed = configs.landing._diff.changed
    flap    = ('wings', 'main_wing', 'control_surfaces', 'flap', 'deflection')
    slat    = ('wings', 'main_wing', 'control_surfaces', 'slat', 'deflection')
    assert abs(changed[flap] - 30. * Units.deg) < 1e-12, f"flap deflection {changed[flap]}"
    assert abs(changed[slat] - 25. * Units.deg) < 1e-12, f"slat deflection {changed[slat]}"
    gear_paths = [path for path in changed if path[-1] == 'gear_extended']
    assert len(gear_paths) > 0 and all(changed[path] == True for path in gear_paths), "landing gear is not extended"

    # the cruise configuration changes nothing, and no configuration adds a value the vehicle does not have
    assert len(configs.cruise._diff.changed) == 0, f"cruise changes {list(configs.cruise._diff.changed)}"
    for config in configs:
        assert len(config._diff.added) == 0, f"{config.tag} adds {config._diff.added}"
    print(f"  landing configuration: {len(changed)} values differ from the vehicle")
    return


def check_applied_vehicle_equals_configuration(vehicle, configs):
    for config in configs:
        applied = deepcopy(vehicle)
        apply_configuration(applied, configuration_of(config))
        left = configuration_values(config, applied)
        assert len(left.changed) == 0, f"{config.tag}: {list(left.changed)[:3]} differ after the configuration is applied"
    print(f"  {len(configs)} configurations: the vehicle with a configuration applied equals the configuration")
    return


def check_order_of_application(vehicle, configs):
    # every configuration of a mission holds a value for the same paths, so the one applied before does not matter
    takeoff, landing = configuration_of(configs.takeoff), configuration_of(configs.landing)
    for path, original in configs.takeoff._diff.original.items():
        landing.vehicle_values.setdefault(path, original)
    for path, original in configs.landing._diff.original.items():
        takeoff.vehicle_values.setdefault(path, original)

    direct = deepcopy(vehicle)
    apply_configuration(direct, landing)
    after_takeoff = deepcopy(vehicle)
    apply_configuration(after_takeoff, takeoff)
    apply_configuration(after_takeoff, landing)
    left = configuration_values(after_takeoff, direct)
    assert len(left.changed) == 0, f"landing after takeoff differs from landing in {list(left.changed)[:3]}"
    print("  landing applied after takeoff equals landing applied alone")
    return


def check_segments_share_one_vehicle(vehicle, mission):
    configurations(mission)
    paths = None
    for segment in mission.segments:
        assert segment.analyses.vehicle is vehicle, f"{segment.tag} does not fly the vehicle of the mission"
        assert segment.analyses.aerodynamics.vehicle is vehicle, f"{segment.tag}: the aerodynamics analysis has another vehicle"
        if paths is None:
            paths = set(segment.configuration.vehicle_values)
        assert set(segment.configuration.vehicle_values) == paths, f"{segment.tag} holds values for other paths"
    tags = sorted(set(segment.configuration.tag for segment in mission.segments))
    print(f"  {len(mission.segments)} segments, {len(tags)} configurations, one vehicle, {len(paths)} values each")
    return


def check_unknown_value_raises(test):
    vehicle = test.vehicle_setup()
    configs = test.configs_setup(vehicle)
    configs.cruise.flap_setting = 1.0
    configs.cruise.store_diff()
    # the error is raised when the segment that flies the configuration is appended to the mission
    try:
        test.mission_setup(test.analyses_setup(configs))
    except ValueError as error:
        assert 'flap_setting' in str(error), f"the error does not name the value: {error}"
        print("  a configuration value the vehicle does not have raises an error that names it")
        return
    raise AssertionError("a configuration value the vehicle does not have was accepted")

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def configuration_of(config):
    """The configuration of a segment that flies this vehicle configuration."""
    configuration                = Data()
    configuration.tag            = config.tag
    configuration.vehicle_values = dict(config._diff.changed)
    return configuration



if __name__ == '__main__':
    main()
