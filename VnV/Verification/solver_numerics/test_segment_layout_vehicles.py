# VnV/Verification/solver_numerics/test_segment_layout_vehicles.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys

import pytest

from RCAIDE.Framework.Mission.Common import Segment_Layout

VNV = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def boeing_787():
    import test_Boeing_787_payload_range as b787
    vehicle  = b787.Boeing_787.vehicle_setup()
    analyses = b787.analyses_setup(b787.Boeing_787.configs_setup(vehicle))
    return b787.missions_setup(b787.payload_range_mission_setup(analyses)).base_mission


def tiltrotor():
    import vtol_aircraft_test as vtol
    vehicle  = vtol.TR_vehicle_setup(False)
    analyses = vtol.TR_analyses_setup(vtol.TR_configs_setup(vehicle))
    return vtol.missions_setup(vtol.TR_mission_setup(analyses)).base_mission


VEHICLES = {"boeing_787": (boeing_787, os.path.join(VNV, "Validation", "aircraft_performance")),
            "tiltrotor":  (tiltrotor,  os.path.join(VNV, "Verification", "network_vtol"))}


@pytest.mark.parametrize("name", VEHICLES)
def test_iterate_writes_into_the_segment_layout(name, monkeypatch):
    """One iterate of every segment replaces no array in the segment's layout and creates no new one: all solver
    data is allocated at initialize and written in place."""
    build, folder = VEHICLES[name]
    monkeypatch.chdir(folder)
    monkeypatch.syspath_prepend(folder)
    monkeypatch.syspath_prepend(os.path.join(VNV, "Vehicles"))
    mission = build()

    mission.process.initialize(mission)
    previous = None
    for tag, segment in mission.segments.items():
        if previous is not None:
            segment.state.initials = previous
        segment.process.initialize.expand_state(segment)
        segment.process.initialize(segment)
        previous = segment.state

        layout = Segment_Layout(segment.state)
        segment.process.iterate(segment)
        assert layout.verify(segment.state) == [], f"{name} {tag}: arrays replaced during iterate"
        assert layout.added(segment.state) == [], f"{name} {tag}: arrays created during iterate"
