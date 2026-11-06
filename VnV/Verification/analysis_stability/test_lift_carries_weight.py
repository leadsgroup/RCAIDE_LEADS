# VnV/Verification/analysis_stability/test_lift_carries_weight.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os

import numpy as np

import RCAIDE
from RCAIDE.Library.Methods.Stability.Common import compute_dynamic_flight_modes, lift_carries_weight

VTOL = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "network_vtol"))


def test_modes_only_where_the_wing_carries_the_vehicle(monkeypatch):
    """Tiltrotor: modes are NaN and the thrust term of CX_u is off while rotor-borne; both apply in wing-borne cruise."""
    monkeypatch.chdir(VTOL)
    monkeypatch.syspath_prepend(VTOL)
    import vtol_aircraft_test as vtol
    vehicle  = vtol.TR_vehicle_setup(False)
    analyses = vtol.TR_analyses_setup(vtol.TR_configs_setup(vehicle))
    results  = vtol.missions_setup(vtol.TR_mission_setup(analyses)).base_mission.evaluate()
    settings = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method().settings

    modes = {}
    for tag in ("departure_transition_1", "cruise"):
        segment = results.segments[tag]
        compute_dynamic_flight_modes(segment.state, settings, segment.analyses.vehicle)
        modes[tag] = segment.state.conditions

    transition, cruise = modes["departure_transition_1"], modes["cruise"]
    assert not np.any(lift_carries_weight(transition, results.segments.departure_transition_1.analyses.vehicle))
    assert np.all(np.isnan(transition.dynamic_stability.LongModes.phugoidDamping))
    assert np.all(np.abs(transition.static_stability.derivatives.CX_u) < 1e-2)    # no thrust term, no blow-up at low q

    assert np.all(lift_carries_weight(cruise, results.segments.cruise.analyses.vehicle))
    assert np.all(np.isfinite(cruise.dynamic_stability.LongModes.phugoidDamping))
    assert np.all(cruise.static_stability.derivatives.CX_u < -0.1)                # thrust term on
