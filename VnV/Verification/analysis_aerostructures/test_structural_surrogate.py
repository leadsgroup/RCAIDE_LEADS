# VnV/Verification/analysis_aerostructures/test_structural_surrogate.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                                       import Data, Units
from RCAIDE.Library.Methods.Performance                                          import aircraft_aerostructural_analysis
from RCAIDE.Library.Mission.Common.Pre_Process                                   import geometry_preprocess_routine
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.evaluate_FEA  import evaluate_surrogate

import numpy as np
import sys
import os

base_dir      = os.path.dirname(os.path.abspath(__file__))
vehicles_path = os.path.abspath(os.path.join(base_dir, "..", "..", "Vehicles"))
if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)

from Boeing_737 import vehicle_setup, configs_setup

# The structural surrogate of the Boeing 737, trained on a small grid, against the direct beam solve at one of its
# training points, where the interpolation is exact. The wing is solved under rigid-wing loads in both.

MACH   = 0.5
ALPHA  = 3.0 * Units.degrees
WINGS  = ('main_wing', 'horizontal_stabilizer')

# ----------------------------------------------------------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    analyses = trained_analyses()
    direct   = direct_solve(analyses.aerodynamics.training.altitude)
    check_surrogate_reproduces_the_direct_solve(analyses, direct)
    check_vertical_wings_are_left_out(analyses)
    check_streamwise_twist_is_continuous(direct)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def build_analyses():
    configs          = configs_setup(vehicle_setup())
    analyses         = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = configs.base
    geometry         = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.settings.compute_fuel_volume = False
    analyses.append(geometry)
    aerodynamics     = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.training.Mach            = np.array([0.3, MACH])
    aerodynamics.training.angle_of_attack = np.array([0.0, 3.0, 6.0]) * Units.degrees
    analyses.append(aerodynamics)
    aerostructures   = RCAIDE.Framework.Analyses.Aerostructures.Finite_Element_Analysis()
    aerostructures.settings.discretization       = 40
    aerostructures.settings.aeroelastic_coupling = 'decoupled'
    analyses.append(aerostructures)
    analyses.append(RCAIDE.Framework.Analyses.Planets.Earth())
    analyses.append(RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976())
    return analyses


def trained_analyses():
    analyses = build_analyses()
    geometry_preprocess_routine(analyses)
    analyses.aerodynamics.initialize(analyses.vehicle, analyses.aerostructures)
    analyses.aerostructures.initialize(analyses.vehicle, analyses.aerodynamics)
    return analyses


def direct_solve(altitude):
    alpha = np.array([[ALPHA]])
    return aircraft_aerostructural_analysis(analyses         = build_analyses(),
                                            angle_of_attacks = alpha,
                                            mach_numbers     = np.ones_like(alpha) * MACH,
                                            altitude         = np.ones_like(alpha) * altitude)


def mission_state(analyses):
    """One control point at the training point, as a mission segment would hold it."""
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976().compute_values(analyses.aerodynamics.training.altitude)
    conditions = RCAIDE.Framework.Mission.Common.Results()
    conditions.aerodynamics.angles.alpha  = np.array([[ALPHA]])
    conditions.freestream.mach_number     = np.array([[MACH]])
    conditions.freestream.density         = np.array([[atmosphere.density[0, 0]]])
    conditions.freestream.velocity        = np.array([[MACH * atmosphere.speed_of_sound[0, 0]]])
    conditions.aerostructures             = Data()
    state            = Data()
    state.conditions = conditions
    state.analyses   = analyses
    return state

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_surrogate_reproduces_the_direct_solve(analyses, direct):
    state = mission_state(analyses)
    evaluate_surrogate(state, analyses.aerostructures.settings, analyses.vehicle)
    for tag in WINGS:
        surrogate = state.conditions.aerostructures[tag]
        for axis in range(3):
            for field in ('deflection', 'rotation'):
                reference = direct[tag][field][0, :, axis]
                error     = np.max(np.abs(surrogate[field][0, :, axis] - reference))
                assert error < 1e-6 * np.max(np.abs(direct[tag][field][0])), (tag, field, axis, error)
        reference = direct[tag].elastic_twist[0, :, 0]
        assert np.max(np.abs(surrogate.elastic_twist[0, :, 0] - reference)) < 1e-6 * np.max(np.abs(reference)), tag
    return


def check_vertical_wings_are_left_out(analyses):
    # the 737 has a vertical tail, which has no beam: no surrogate and no results
    state = mission_state(analyses)
    evaluate_surrogate(state, analyses.aerostructures.settings, analyses.vehicle)
    assert 'vertical_stabilizer' not in analyses.aerostructures.surrogates.subsonic.deflection_w
    assert 'vertical_stabilizer' not in analyses.aerostructures.surrogates.subsonic.rotation_y
    assert 'vertical_stabilizer' not in state.conditions.aerostructures
    return


def check_streamwise_twist_is_continuous(direct):
    # the projection of the rotation on the local beam axis jumps where the beam's sweep changes, at the raked tip of
    # the 737 wing; the rotation about y, the streamwise twist, does not
    wing            = direct.main_wing
    streamwise_step = np.max(np.abs(np.diff(wing.rotation[0, :, 1])))
    projected_step  = np.max(np.abs(np.diff(wing.elastic_twist[0, :, 0])))
    assert streamwise_step < 0.1 * projected_step, (streamwise_step, projected_step)
    return


if __name__ == '__main__':
    main()
    print('test_structural_surrogate passed')
