# VnV/Verification/analysis_aerodynamics/test_surrogate_inputs.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys
import numpy as np

from RCAIDE.Framework.Core import Data, DataOrdered, Units

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # The aerodynamic surrogates are the expensive part of a mission setup. They are kept when the setup is repeated
    # and nothing they were built from has changed (surrogate_inputs), and built again when the wing changes.
    # Checked on the Boeing 737 mission of the turbofan network test with a coarse mesh and few training points.
    vnv_folder   = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    start_folder = os.getcwd()
    sys.path.insert(0, os.path.join(vnv_folder, "Vehicles"))
    os.chdir(os.path.join(vnv_folder, "Verification", "network_turbofan"))
    sys.path.insert(0, os.getcwd())

    mission = coarse_mission()
    check_reads_are_declared(mission)
    check_kept_when_nothing_changed(mission)
    check_kept_when_the_mission_changes(mission)
    check_built_again_when_the_wing_changes(mission)

    os.chdir(start_folder)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_reads_are_declared(mission):
    # every vehicle value the surrogate build reads must be one of its declared inputs; names and the control
    # surface deflections the training sets itself are the exceptions
    vehicle = first_segment(mission).analyses.vehicle
    for stage in ('configurations', 'hp_decompose_mission', 'geometry', 'energy', 'mass_properties'):
        mission.process.initialize[stage](mission)
    reads    = traced_reads(vehicle, lambda: mission.process.initialize.aero(mission))
    declared = set(aerodynamics_of(mission).surrogate_inputs)
    missing  = sorted(path for path in reads if path not in declared
                      and path.split('.')[-1] not in ('tag', 'deflection', 'secondary_deflection'))
    assert len(reads) > 50, f"only {len(reads)} vehicle values were traced"
    assert len(missing) == 0, f"the surrogate build reads values that are not surrogate inputs: {missing[:10]}"
    print(f"  the surrogate build reads {len(reads)} vehicle values, all among its {len(declared)} inputs")
    return


def check_kept_when_nothing_changed(mission):
    tables = trained_tables(mission)
    mission.process.initialize(mission)
    assert trained_tables(mission) is tables, "the surrogates were built again although nothing changed"
    print("  setup repeated: surrogates kept")
    return


def check_kept_when_the_mission_changes(mission):
    tables = trained_tables(mission)
    mission.segments.cruise_1.distance *= 1.1
    mission.process.initialize(mission)
    assert trained_tables(mission) is tables, "the surrogates were built again for a change of cruise distance"
    print("  cruise distance changed: surrogates kept")
    return


def check_built_again_when_the_wing_changes(mission):
    tables  = trained_tables(mission)
    vehicle = first_segment(mission).analyses.vehicle
    for segment in vehicle.wings.main_wing.segments:
        segment.twist += 0.5 * Units.degrees
    mission.process.initialize(mission)
    assert trained_tables(mission) is not tables, "the surrogates were kept although the wing twist changed"
    print("  wing twist changed: surrogates built again")
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def coarse_mission():
    import turbofan_network_test as test
    analyses = test.analyses_setup(test.configs_setup(test.vehicle_setup()))
    for configuration_analyses in analyses:
        aerodynamics = configuration_analyses.aerodynamics
        aerodynamics.settings.number_of_spanwise_vortices  = 12
        aerodynamics.settings.number_of_chordwise_vortices = 3
        aerodynamics.training.angle_of_attack              = np.array([-2., 2., 8.]) * Units.deg
        aerodynamics.training.Mach                         = np.array([0.1, 0.5, 0.85])
    return test.missions_setup(test.mission_setup(analyses)).base_mission


def first_segment(mission):
    return list(mission.segments.values())[0]


def aerodynamics_of(mission):
    """The analysis that owns the surrogates the segments share."""
    aerodynamics = first_segment(mission).analyses.aerodynamics
    return aerodynamics.surrogate_owner if aerodynamics.surrogate_owner is not None else aerodynamics


def trained_tables(mission):
    return aerodynamics_of(mission).training.subsonic


def traced_reads(vehicle, run):
    """Paths of the vehicle values read while run() executes, through attribute and item access of Data."""
    paths = {}
    container_paths(vehicle, '', paths)
    reads = set()

    original_getattribute = Data.__getattribute__
    def traced_getattribute(self, key):
        value = original_getattribute(self, key)
        record(self, key, value)
        return value
    def traced_getitem(self, key):
        value = dict.__getitem__(self, key)
        record(self, key, value)
        return value
    def record(container, key, value):
        path = paths.get(id(container))
        if path is not None and not isinstance(value, dict) and not callable(value) and not str(key).startswith('_'):
            reads.add(f"{path}{key}")

    Data.__getattribute__ = traced_getattribute
    Data.__getitem__      = traced_getitem
    try:
        run()
    finally:
        Data.__getattribute__ = original_getattribute
        del Data.__getitem__
    return reads


def container_paths(item, path, paths):
    """The path of every Data container below item, by the id of the container."""
    if not isinstance(item, (Data, DataOrdered)) or id(item) in paths:
        return
    paths[id(item)] = path
    for key in list(dict.keys(item)):
        if not str(key).startswith('_'):
            container_paths(dict.__getitem__(item, key), f"{path}{key}.", paths)
    return


if __name__ == '__main__':
    main()
