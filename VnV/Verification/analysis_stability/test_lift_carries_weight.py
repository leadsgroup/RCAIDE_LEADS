# VnV/Verification/analysis_stability/test_lift_carries_weight.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys

import numpy as np

import RCAIDE
from RCAIDE.Library.Methods.Stability.Common import compute_dynamic_flight_modes, lift_carries_weight

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # The stability derivatives have no rotor terms, so dynamic modes only apply where aerodynamic lift carries the
    # vehicle. A tiltrotor flies both ways: rotor-borne in transition and wing-borne in cruise.
    check_modes_only_where_lift_carries_the_weight()
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_modes_only_where_lift_carries_the_weight():
    # the imported VnV script expects to run from its own folder
    start_folder = os.getcwd()
    test_folder  = os.path.dirname(os.path.abspath(__file__))
    os.chdir(os.path.join(test_folder, "..", "network_vtol"))
    sys.path.insert(0, os.getcwd())
    sys.path.insert(0, os.path.join(test_folder, "..", "..", "Vehicles"))
    import vtol_aircraft_test as vtol

    vehicle  = vtol.TR_vehicle_setup(False)
    analyses = vtol.TR_analyses_setup(vtol.TR_configs_setup(vehicle))
    results  = vtol.missions_setup(vtol.TR_mission_setup(analyses)).base_mission.evaluate()
    settings = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method().settings
    os.chdir(start_folder)

    modes = {}
    for tag in ("departure_transition_1", "cruise"):
        segment = results.segments[tag]
        compute_dynamic_flight_modes(segment.state, settings, segment.analyses.vehicle)
        modes[tag] = segment.state.conditions
    transition, cruise = modes["departure_transition_1"], modes["cruise"]

    # rotor-borne: modes are NaN, and CX_u has no thrust term, so it does not blow up at low dynamic pressure
    assert not np.any(lift_carries_weight(transition, results.segments.departure_transition_1.analyses.vehicle))
    assert np.all(np.isnan(transition.dynamic_stability.LongModes.phugoidDamping))
    assert np.all(np.abs(transition.static_stability.derivatives.CX_u) < 1e-2)

    # wing-borne: modes are computed, and CX_u includes the thrust term
    assert np.all(lift_carries_weight(cruise, results.segments.cruise.analyses.vehicle))
    assert np.all(np.isfinite(cruise.dynamic_stability.LongModes.phugoidDamping))
    assert np.all(cruise.static_stability.derivatives.CX_u < -0.1)
    return


if __name__ == '__main__':
    main()
