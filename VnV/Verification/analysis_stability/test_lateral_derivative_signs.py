# VnV/Verification/analysis_stability/test_lateral_derivative_signs.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys

import numpy as np

import RCAIDE
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.train_VLM_surrogates import call_VLM
from RCAIDE.Library.Mission.Common.Pre_Process import geometry

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # The signs of the VLM's lateral derivatives must match AVL, which follows Etkin & Reid's axes.
    # The Navion has a fin and the tiltrotor has a V-tail, and the two used to disagree.
    # beta is sideslip, p is roll rate and r is yaw rate.
    # AVL 3.35 (Drela & Youngren) on the same panels at alpha = 2 deg, Mach 0.15; columns are CY, Cl, Cn
    navion_AVL    = {"beta": (-0.2123, -0.1201, 0.0840), "p": (-0.1544, -0.4304, -0.0135), "r": (0.2145, 0.0664, -0.1025)}
    tiltrotor_AVL = {"beta": (-0.3759, -0.1469, 0.1198), "p": (-0.2039, -0.5674,  0.0351), "r": (0.3113, 0.1291, -0.1054)}

    start_folder = os.getcwd()
    test_folder  = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(test_folder, "..", "..", "Vehicles"))

    # the imported VnV scripts expect to run from their own folders
    os.chdir(test_folder)
    sys.path.insert(0, os.getcwd())
    check_lateral_signs_match_AVL("navion", navion_mission(), navion_AVL)

    os.chdir(os.path.join(test_folder, "..", "network_vtol"))
    sys.path.insert(0, os.getcwd())
    check_lateral_signs_match_AVL("tiltrotor", tiltrotor_mission(), tiltrotor_AVL)

    os.chdir(start_folder)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_lateral_signs_match_AVL(name, mission, AVL):
    geometry(mission)
    vehicle  = list(mission.segments.values())[-1].analyses.vehicle
    settings = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method().settings

    # a central difference in each variable; the rate step is 0.05 in nondimensional rate, p b / (2 V)
    rate_step = 0.05 * 2 * 0.15 * 343. / vehicle.reference_span
    for variable, step in (("beta", 0.05), ("p", rate_step), ("r", rate_step)):
        plus   = lateral_coefficients(vehicle, settings, **{variable: step})
        minus  = lateral_coefficients(vehicle, settings, **{variable: -step})
        change = plus - minus
        assert np.array_equal(np.sign(change), np.sign(AVL[variable])), f"{name} {variable}: {change}"
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def lateral_coefficients(vehicle, settings, beta=0., p=0., r=0.):
    """Side force, rolling moment and yawing moment coefficients at alpha = 2 deg, Mach 0.15."""
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


def navion_mission():
    import trimmed_flight_test as test
    import Navion
    analyses = test.analyses_setup(Navion.configs_setup(Navion.vehicle_setup()))
    return test.missions_setup(test.mission_setup(analyses)).base_mission


def tiltrotor_mission():
    import vtol_aircraft_test as test
    analyses = test.TR_analyses_setup(test.TR_configs_setup(test.TR_vehicle_setup(False)))
    return test.missions_setup(test.TR_mission_setup(analyses)).base_mission


if __name__ == '__main__':
    main()
