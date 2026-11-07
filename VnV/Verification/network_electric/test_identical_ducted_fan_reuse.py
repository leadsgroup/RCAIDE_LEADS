# VnV/Verification/network_electric/test_identical_ducted_fan_reuse.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Vehicles"))
import electric_ducted_fan_aircraft_test as EDF
from NASA_X48 import vehicle_setup, configs_setup

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # With identical_propulsors, the wing fans reuse the center fan's results instead of computing their own.
    # The mission must give the same answer either way.
    check_reuse_matches_computing_each_fan()
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_reuse_matches_computing_each_fan():
    # with reuse off each fan has its own unknowns, so the runs agree to the solver tolerance (1e-6), not exactly
    for reused, computed in zip(fly(identical=True), fly(identical=False)):
        for path in ("energy.outputs.power.propulsive", "frames.body.total_force_vector",
                     "frames.inertial.position_vector"):
            assert np.allclose(reused.deep_get(path), computed.deep_get(path), rtol=1e-5, atol=1e-8), path
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def fly(identical):
    """The conditions of each segment of the X-48 VnV mission with Rankine-Froude ducted fans."""
    vehicle = vehicle_setup(True, 'Rankine_Froude_Momentum_Theory')
    for propulsor in list(vehicle.networks.values())[0].propulsors:
        propulsor.identical_propulsors = identical
    analyses = EDF.analyses_setup(configs_setup(vehicle))
    results  = EDF.missions_setup(EDF.mission_setup(analyses)).base_mission.evaluate()
    return [segment.state.conditions for segment in results.segments]


if __name__ == '__main__':
    main()
