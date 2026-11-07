# VnV/Verification/analysis_stability/test_crosswind_sideslip.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys

import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # A crosswind sets the sideslip angle beta from the airspeed at each control point, beta = arcsin(v/V), Etkin & Reid
    # (1.6,4). This must work on every segment type, including those with no air_speed input: a 787 cruise inherits its
    # speed from the previous segment, and climb and transition segments define speed in other ways.
    start_folder = os.getcwd()
    test_folder  = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(test_folder, "..", "..", "Vehicles"))

    # the imported VnV scripts expect to run from their own folders
    os.chdir(os.path.join(test_folder, "..", "..", "Validation", "aircraft_performance"))
    sys.path.insert(0, os.getcwd())
    check_crosswind_sets_sideslip(boeing_787_mission())

    os.chdir(os.path.join(test_folder, "..", "network_vtol"))
    sys.path.insert(0, os.getcwd())
    check_crosswind_sets_sideslip(tiltrotor_mission())

    os.chdir(start_folder)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_crosswind_sets_sideslip(mission):
    crosswind_speed = 1.0   # [m/s]

    # initialize each segment from the previous one without solving, then iterate twice so that speeds set during an
    # iterate are already in place on the second one
    mission.process.initialize(mission)
    previous = None
    for tag, segment in mission.segments.items():
        if previous is not None:
            segment.state.initials = previous
        previous = segment.state
        segment.process.initialize.expand_state(segment)
        segment.process.initialize(segment)
        segment.crosswind_speed = crosswind_speed
        segment.process.iterate(segment)
        segment.process.iterate(segment)

        # the ground model has no sideways motion, so takeoff and landing do not take a crosswind
        if any(getattr(step, "__name__", "") == "ground" for step in segment.process.iterate.unknowns.mission.values()):
            continue

        # beta follows the airspeed at each control point, and is zero where the vehicle is not moving
        V        = np.linalg.norm(segment.state.conditions.frames.inertial.velocity_vector, axis=1)
        beta     = segment.state.conditions.frames.wind.body_rotations[:, 2]
        expected = np.arcsin(np.clip(np.divide(crosswind_speed, V, out=np.zeros_like(V), where=V > 0.), -1.0, 1.0))
        assert np.allclose(beta, expected, rtol=1e-12, atol=1e-12), f"{tag}: beta {beta[:2]}, expected {expected[:2]}"
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
