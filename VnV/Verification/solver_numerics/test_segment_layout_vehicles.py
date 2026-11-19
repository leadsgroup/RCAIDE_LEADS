# VnV/Verification/solver_numerics/test_segment_layout_vehicles.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys

from RCAIDE.Framework.Mission.Common import Segment_Layout

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # Segment_Layout places a segment's unknowns, residuals and conditions in one preallocated buffer.
    # This checks the rule it relies on, on real vehicles: all solver data is allocated at initialize and written in
    # place, so one iterate of any segment replaces no array and creates no new one.
    vnv_folder = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    sys.path.insert(0, os.path.join(vnv_folder, "Vehicles"))
    start_folder = os.getcwd()

    # the imported VnV scripts expect to run from their own folders
    os.chdir(os.path.join(vnv_folder, "Validation", "aircraft_performance"))
    sys.path.insert(0, os.getcwd())
    check_iterate_writes_in_place("boeing_787", boeing_787_mission())

    os.chdir(os.path.join(vnv_folder, "Verification", "network_vtol"))
    sys.path.insert(0, os.getcwd())
    check_iterate_writes_in_place("tiltrotor", tiltrotor_mission())

    os.chdir(start_folder)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_iterate_writes_in_place(name, mission):
    mission.process.initialize(mission)

    # initialize each segment from the previous one, as a mission solve does, without solving it
    previous = None
    for tag, segment in mission.segments.items():
        if previous is not None:
            segment.state.initials = previous
        segment.process.initialize.expand_state(segment)
        segment.process.initialize(segment)
        previous = segment.state

        # verify lists arrays replaced since the layout was built; added lists arrays created since then
        layout = Segment_Layout(segment.state)
        segment.process.iterate(segment)
        assert layout.verify(segment.state) == [], f"{name} {tag}: arrays replaced during iterate"
        assert layout.added(segment.state) == [], f"{name} {tag}: arrays created during iterate"
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Missions
# ----------------------------------------------------------------------------------------------------------------------
def boeing_787_mission():
    import test_Boeing_787_payload_range as b787
    vehicle  = b787.Boeing_787.vehicle_setup()
    analyses = b787.analyses_setup(b787.Boeing_787.configs_setup(vehicle))
    return b787.missions_setup(b787.payload_range_mission_setup(analyses)).base_mission


def tiltrotor_mission():
    import vtol_aircraft_test as vtol
    vehicle  = vtol.TR_vehicle_setup(False)
    analyses = vtol.TR_analyses_setup(vtol.TR_configs_setup(vehicle))
    return vtol.missions_setup(vtol.TR_mission_setup(analyses)).base_mission


if __name__ == '__main__':
    main()
