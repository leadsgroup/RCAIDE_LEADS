# VnV/Verification/network_electric/test_identical_ducted_fan_reuse.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

import electric_ducted_fan_aircraft_test as EDF
from NASA_X48 import vehicle_setup, configs_setup


def fly(identical):
    """The X-48 VnV mission with Rankine-Froude ducted fans; identical sets whether the wing fans reuse the center
    fan's results."""
    vehicle = vehicle_setup(True, 'Rankine_Froude_Momentum_Theory')
    for propulsor in list(vehicle.networks.values())[0].propulsors:
        propulsor.identical_propulsors = identical
    analyses = EDF.analyses_setup(configs_setup(vehicle))
    results  = EDF.missions_setup(EDF.mission_setup(analyses)).base_mission.evaluate()
    return [segment.state.conditions for segment in results.segments]


def test_reuse_matches_computing_each_fan():
    # with reuse off each fan has its own unknowns, so the runs agree to the solver tolerance (1e-6)
    for reused, computed in zip(fly(identical=True), fly(identical=False)):
        for path in ("energy.outputs.power.propulsive", "frames.body.total_force_vector",
                     "frames.inertial.position_vector"):
            assert np.allclose(reused.deep_get(path), computed.deep_get(path), rtol=1e-5, atol=1e-8), path
