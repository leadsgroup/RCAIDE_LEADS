# VnV/Verification/analysis_stability/test_compute_neutral_point.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                         import Units, Data
from RCAIDE.Library.Methods.Stability.Vortex_Lattice_Method        import compute_neutral_point
from RCAIDE.Library.Methods.Stability.Vortex_Lattice_Method.compute_neutral_point import FUSELAGE_MOMENT_FACTOR_POSITION, FUSELAGE_MOMENT_FACTOR
from RCAIDE.Library.Mission.Common.Pre_Process.geometry            import geometry_preprocess_routine
from copy import deepcopy
import numpy as np
import sys
import os

sys.path.append(os.path.join(os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Embraer_190 import vehicle_setup

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def e190():
    vehicle  = vehicle_setup()
    analyses = RCAIDE.Framework.Analyses.Vehicle(); analyses.vehicle = vehicle
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry(); geometry.vehicle = vehicle; analyses.append(geometry)
    geometry_preprocess_routine(analyses)
    return vehicle


def neutral_point(vehicle, fuselage_correction=True):
    stability          = Data()
    stability.settings = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method().settings
    stability.settings.fuselage_correction = fuselage_correction
    stability.training = Data()
    stability.training.angle_of_attack = np.array([-2., 0., 1., 3.]) * Units.deg
    stability.training.Mach            = np.array([0.3])
    compute_neutral_point(stability, vehicle)
    return vehicle.neutral_point

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def test_fuselage_moment_factor_is_naca_report_711_figure_8():
    # Gilruth and White, NACA Report 711, Figure 8: K_f = 1.0 at 40% body length, increasing with wing position
    assert np.isclose(np.interp(40., FUSELAGE_MOMENT_FACTOR_POSITION, FUSELAGE_MOMENT_FACTOR), 0.989)
    assert np.all(np.diff(FUSELAGE_MOMENT_FACTOR) > 0)


def test_neutral_point_is_smooth_in_tail_position():
    # moving the horizontal tail 0.3 m aft moves the neutral point aft by a small, smooth amount
    vehicle  = e190()
    MAC      = vehicle.wings.main_wing.chords.mean_aerodynamic
    base     = neutral_point(deepcopy(vehicle))
    for dx in (0.3, -0.3):
        moved = deepcopy(vehicle)
        moved.wings.horizontal_stabilizer.origin[0][0] += dx
        shift = (neutral_point(moved) - base) / MAC
        assert 0. < np.sign(dx) * shift < 0.05


def test_fuselage_moves_neutral_point_forward():
    # the fuselage term is destabilizing: removing the fuselage moves the neutral point aft
    vehicle     = e190()
    no_fuselage = deepcopy(vehicle)
    for tag in list(no_fuselage.fuselages.keys()):
        del no_fuselage.fuselages[tag]
    assert neutral_point(no_fuselage) > neutral_point(deepcopy(vehicle))


def test_fuselage_correction_matches_fuselage_panels_on_e190():
    # where the VLM fuselage panels behave (E190), both settings agree within 2% MAC (62.3% vs 63.0% MAC)
    vehicle = e190()
    MAC     = vehicle.wings.main_wing.chords.mean_aerodynamic
    assert abs(neutral_point(deepcopy(vehicle), True) - neutral_point(deepcopy(vehicle), False)) / MAC < 0.02


def main():
    test_fuselage_moment_factor_is_naca_report_711_figure_8()
    test_neutral_point_is_smooth_in_tail_position()
    test_fuselage_moves_neutral_point_forward()
    test_fuselage_correction_matches_fuselage_panels_on_e190()
    return


if __name__ == '__main__':
    main()
