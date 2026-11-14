# VnV/Verification/analysis_weights/test_mass_properties_repeatable.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys
import numpy as np

from RCAIDE.Library.Mission.Common.Pre_Process.mass_properties import mass_properties_preprocess_routine

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # The mission setup estimates the weights of the aircraft of every segment. One aircraft has one empty weight: it
    # must be the same in every segment of a mission, and estimating it a second time must not change it. Checked on
    # a turboprop transport (FLOPS, whose unusable fuel depends on the fuel capacity) and on a stopped-rotor eVTOL
    # (component masses set by the user).
    vnv_folder   = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    start_folder = os.getcwd()
    sys.path.insert(0, os.path.join(vnv_folder, "Vehicles"))

    os.chdir(os.path.join(vnv_folder, "Verification", "network_turboprop"))
    sys.path.insert(0, os.getcwd())
    mission = turboprop_mission()
    check_same_weights_in_every_segment("ATR 72", mission)
    check_second_estimate_unchanged("ATR 72", mission)

    os.chdir(os.path.join(vnv_folder, "Verification", "network_vtol"))
    sys.path.insert(0, os.getcwd())
    mission = stopped_rotor_mission()
    check_same_weights_in_every_segment("stopped rotor", mission)
    check_second_estimate_unchanged("stopped rotor", mission)

    os.chdir(start_folder)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_same_weights_in_every_segment(name, mission):
    segments = list(mission.segments.values())
    first    = weight_values(segments[0])
    for segment in segments[1:]:
        values = weight_values(segment)
        for key in first:
            assert abs(values[key] - first[key]) <= 1e-9 * abs(first[key]), \
                   f"{name} {segment.tag}: {key} {values[key]} against {first[key]} in the first segment"
    print(f"  {name}: empty weight {first['operating empty']:.3f} kg in all {len(segments)} segments")
    return


def check_second_estimate_unchanged(name, mission):
    segment = list(mission.segments.values())[0]
    first   = weight_values(segment)
    mass_properties_preprocess_routine(segment, 0)
    second  = weight_values(segment)
    for key in first:
        assert abs(second[key] - first[key]) <= 1e-9 * abs(first[key]), f"{name}: {key} {first[key]} then {second[key]} when estimated again"
    print(f"  {name}: a second estimate leaves {len(first)} weights unchanged")
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Missions
# ----------------------------------------------------------------------------------------------------------------------
def weight_values(segment):
    mass_properties = segment.analyses.vehicle.mass_properties
    breakdown       = mass_properties.weight_breakdown
    return {'operating empty':   float(mass_properties.operating_empty),
            'empty':             float(breakdown.empty.total),
            'structural':        float(breakdown.empty.structural.total),
            'propulsion':        float(breakdown.empty.propulsion.total),
            'systems':           float(breakdown.empty.systems.total),
            'operational items': float(breakdown.operational_items.total)}


def set_up_weights(mission):
    """The steps of the mission setup up to the mass properties, without the aerodynamic surrogate."""
    for step in ('hp_decompose_mission', 'geometry', 'energy', 'mass_properties'):
        mission.process.initialize[step](mission)
    return mission


def turboprop_mission():
    import turboprop_network_test as test
    analyses = test.analyses_setup(test.configs_setup(test.vehicle_setup()))
    return set_up_weights(test.missions_setup(test.mission_setup(analyses)).base_mission)


def stopped_rotor_mission():
    import vtol_aircraft_test as test
    vehicle  = test.SR_vehicle_setup(False)
    analyses = test.SR_analyses_setup(test.SR_configs_setup(vehicle))
    return set_up_weights(test.missions_setup(test.SR_mission_setup(analyses, vehicle)).base_mission)


if __name__ == '__main__':
    main()
