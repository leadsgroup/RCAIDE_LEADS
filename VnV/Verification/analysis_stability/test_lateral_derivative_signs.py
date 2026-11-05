# VnV/Verification/analysis_stability/test_lateral_derivative_signs.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys

import numpy as np
import pytest

import RCAIDE
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.train_VLM_surrogates import call_VLM
from RCAIDE.Library.Mission.Common.Pre_Process import geometry

VNV = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

# AVL 3.35 (Drela & Youngren) on the same panels at alpha = 2 deg, Mach 0.15; columns CY, Cl, Cn
AVL = {"navion":    {"beta": (-0.2123, -0.1201, 0.0840), "p": (-0.1544, -0.4304, -0.0135), "r": (0.2145, 0.0664, -0.1025)},
       "tiltrotor": {"beta": (-0.3759, -0.1469, 0.1198), "p": (-0.2039, -0.5674,  0.0351), "r": (0.3113, 0.1291, -0.1054)}}


def navion():
    import trimmed_flight_test as test
    import Navion
    analyses = test.analyses_setup(Navion.configs_setup(Navion.vehicle_setup()))
    return test.missions_setup(test.mission_setup(analyses)).base_mission


def tiltrotor():
    import vtol_aircraft_test as test
    analyses = test.TR_analyses_setup(test.TR_configs_setup(test.TR_vehicle_setup(False)))
    return test.missions_setup(test.TR_mission_setup(analyses)).base_mission


VEHICLES = {"navion":    (navion,    os.path.join(VNV, "Verification", "analysis_stability")),
            "tiltrotor": (tiltrotor, os.path.join(VNV, "Verification", "network_vtol"))}


def lateral_coefficients(vehicle, settings, beta=0., p=0., r=0.):
    conditions = RCAIDE.Framework.Mission.Common.Results()
    conditions.expand_rows(1, override=False)
    conditions.freestream.mach_number      = np.array([[0.15]])
    conditions.freestream.velocity         = np.array([[0.15 * 343.]])
    conditions.aerodynamics.angles.alpha   = np.array([[np.radians(2.)]])
    conditions.aerodynamics.angles.beta    = np.array([[beta]])
    conditions.static_stability.pitch_rate = np.array([[0.]])
    conditions.static_stability.roll_rate  = np.array([[p]])
    conditions.static_stability.yaw_rate   = np.array([[r]])
    results = call_VLM(conditions, settings, vehicle)
    return np.array([results.CY[0, 0], results.CL[0, 0], results.CN[0, 0]])


@pytest.mark.parametrize("name", VEHICLES)
def test_lateral_signs_match_avl(name, monkeypatch):
    """A fin (Navion) and a V-tail (tiltrotor) both give the signs of AVL, which follows Etkin & Reid's axes."""
    build, folder = VEHICLES[name]
    monkeypatch.chdir(folder)
    monkeypatch.syspath_prepend(folder)
    monkeypatch.syspath_prepend(os.path.join(VNV, "Vehicles"))
    mission = build()
    geometry(mission)
    vehicle  = list(mission.segments.values())[-1].analyses.vehicle
    settings = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method().settings

    rate = 0.05 * 2 * 0.15 * 343. / vehicle.reference_span  # 0.05 in nondimensional rate
    for variable, step in (("beta", 0.05), ("p", rate), ("r", rate)):
        change = lateral_coefficients(vehicle, settings, **{variable: step}) - lateral_coefficients(vehicle, settings, **{variable: -step})
        assert np.array_equal(np.sign(change), np.sign(AVL[name][variable])), f"{name} {variable}: {change}"
