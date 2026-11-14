# VnV/Verification/analysis_aerostructures/test_static_aeroelastic_model.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                                                   import Units
from RCAIDE.Library.Methods.Performance                                                      import aircraft_aerostructural_analysis
from RCAIDE.Library.Mission.Common.Pre_Process                                               import geometry_preprocess_routine
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.static_aeroelastic_model  import build_static_aeroelastic_model, solve_static_aeroelastic_model

from copy import deepcopy
import numpy as np
import sys
import os

base_dir      = os.path.dirname(os.path.abspath(__file__))
vehicles_path = os.path.abspath(os.path.join(base_dir, "..", "..", "Vehicles"))
if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)

from Boeing_737 import vehicle_setup, configs_setup

# flight condition of the checks
MACH       = 0.5
ALTITUDE   = 3000.0
ALPHA      = 3.0 * Units.degrees

# fuel [kg] in the two wing tanks; different so a mix-up between tanks shows
FUEL_MASSES = {'integral_tank': 5000.0, 'integral_tank2': 2000.0}

# ----------------------------------------------------------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    model = build_model()
    check_one_way_matches_the_beam_solve(model)
    check_coupled_matches_the_coupled_iteration(model)
    check_fuel_enters_by_tank(model)
    check_feedback_vanishes_without_dynamic_pressure(model)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def build_analyses(coupled):
    """Boeing 737 whose two wing tanks each carry their own fuel, so the direct solve reads each tank's mass."""
    vehicle = vehicle_setup()
    for network in vehicle.networks:
        for source in network.sources:
            if source.tag in FUEL_MASSES:
                source.fuel                      = deepcopy(source.fuel)
                source.fuel.tag                  = source.tag + '_fuel'
                source.fuel.mass_properties.mass = FUEL_MASSES[source.tag]
    configs = configs_setup(vehicle)

    analyses         = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = configs.base
    geometry         = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.settings.compute_fuel_volume = False
    analyses.append(geometry)
    analyses.append(RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method())
    aerostructures = RCAIDE.Framework.Analyses.Aerostructures.Finite_Element_Analysis()
    aerostructures.settings.discretiation        = 100
    aerostructures.settings.aeroelastic_coupling = 'coupled' if coupled else 'decoupled'
    analyses.append(aerostructures)
    analyses.append(RCAIDE.Framework.Analyses.Planets.Earth())
    analyses.append(RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976())
    return analyses


def build_model():
    """Static aeroelastic model with the flight condition on its Mach and angle of attack tables."""
    analyses = build_analyses(coupled=True)
    geometry_preprocess_routine(analyses)
    return build_static_aeroelastic_model(analyses.vehicle, analyses.aerodynamics.settings, analyses.aerostructures.settings,
                                          mach_numbers=[MACH], angles_of_attack=np.array([0.0, 3.0, 6.0]) * Units.degrees,
                                          number_of_stations=4)


def direct_solve(coupled):
    alpha = np.array([[ALPHA]])
    return aircraft_aerostructural_analysis(analyses         = build_analyses(coupled),
                                            angle_of_attacks = alpha,
                                            mach_numbers     = np.ones_like(alpha) * MACH,
                                            altitude         = np.ones_like(alpha) * ALTITUDE)


def dynamic_pressure():
    atmo = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976().compute_values(ALTITUDE)
    return 0.5 * atmo.density[0, 0] * (MACH * atmo.speed_of_sound[0, 0])**2

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def check_one_way_matches_the_beam_solve(model):
    # without feedback the model is the beam solve under rigid loads, structure, engines and fuel, on every wing
    direct = direct_solve(coupled=False)
    result = solve_static_aeroelastic_model(model, MACH, dynamic_pressure(), ALPHA, fuel_masses=FUEL_MASSES, coupled=False)
    for tag in model.wings.keys():
        scale = np.max(np.abs(direct[tag].deflection[0]))
        error = np.max(np.abs(result[tag].deflection - direct[tag].deflection[0])) / scale
        assert error < 1e-6, (tag, error)
        scale = np.max(np.abs(direct[tag].rotation[0]))
        error = np.max(np.abs(result[tag].rotation - direct[tag].rotation[0])) / scale
        assert error < 1e-6, (tag, error)

        # and so are the stresses recovered from it
        for field in ('normal_stress', 'shear_stress'):
            scale = np.max(np.abs(direct[tag][field][0]))
            error = np.max(np.abs(result[tag][field] - direct[tag][field][0])) / scale
            assert error < 1e-6, (tag, field, error)
        assert np.allclose(result[tag].margin_of_safety, direct[tag].margin_of_safety[0], rtol=1e-6)
    return


def check_coupled_matches_the_coupled_iteration(model):
    # with feedback the model matches the fixed-point iteration that rebuilds the mesh
    direct = direct_solve(coupled=True)
    result = solve_static_aeroelastic_model(model, MACH, dynamic_pressure(), ALPHA, fuel_masses=FUEL_MASSES, coupled=True)

    tip_deflection        = result.main_wing.deflection[-1, 2]
    direct_tip_deflection = direct.main_wing.deflection[0, -1, 2]
    assert abs(tip_deflection - direct_tip_deflection) < 0.01 * abs(direct_tip_deflection), (tip_deflection, direct_tip_deflection)

    tip_twist        = result.main_wing.rotation[-1, 1]
    direct_tip_twist = direct.main_wing.rotation[0, -1, 1]
    assert abs(tip_twist - direct_tip_twist) < 0.02 * abs(direct_tip_twist), (tip_twist, direct_tip_twist)

    lift_coefficient        = result.CLift
    direct_lift_coefficient = direct.lift_coefficient[0, 0]
    assert abs(lift_coefficient - direct_lift_coefficient) < 0.01 * abs(direct_lift_coefficient), (lift_coefficient, direct_lift_coefficient)

    peak_stress        = np.max(result.main_wing.normal_stress)
    direct_peak_stress = np.max(direct.main_wing.normal_stress[0])
    assert abs(peak_stress - direct_peak_stress) < 0.01 * direct_peak_stress, (peak_stress, direct_peak_stress)

    # the feedback is not negligible here, so the agreement above is a real check
    one_way = solve_static_aeroelastic_model(model, MACH, dynamic_pressure(), ALPHA, fuel_masses=FUEL_MASSES, coupled=False)
    assert abs(one_way.main_wing.deflection[-1, 2] - tip_deflection) > 0.03 * abs(tip_deflection)
    return


def check_fuel_enters_by_tank(model):
    # the response is linear in each tank's fuel, and the two tanks do not act alike
    q = dynamic_pressure()
    def tip(fuel_masses):
        return solve_static_aeroelastic_model(model, MACH, q, ALPHA, fuel_masses=fuel_masses, coupled=False).main_wing.deflection[-1, 2]
    empty    = tip({})
    inboard  = tip({'integral_tank': 1000.0}) - empty
    outboard = tip({'integral_tank2': 1000.0}) - empty
    both     = tip({'integral_tank': 3000.0, 'integral_tank2': 2000.0}) - empty
    assert inboard < 0.0 and outboard < 0.0                 # fuel in the wing relieves the upward bending
    assert abs(outboard) > abs(inboard)                     # more so further outboard
    assert abs(both - (3 * inboard + 2 * outboard)) < 1e-10 * abs(both)
    return


def check_feedback_vanishes_without_dynamic_pressure(model):
    # at zero dynamic pressure only the inertial loads remain and the coupled and one-way responses agree
    coupled = solve_static_aeroelastic_model(model, MACH, 0.0, ALPHA, fuel_masses=FUEL_MASSES, coupled=True)
    one_way = solve_static_aeroelastic_model(model, MACH, 0.0, ALPHA, fuel_masses=FUEL_MASSES, coupled=False)
    assert np.allclose(coupled.main_wing.deflection, one_way.main_wing.deflection, rtol=0, atol=1e-14)
    assert coupled.main_wing.deflection[-1, 2] < 0.0        # the wing sags under its own weight, engines and fuel
    return


if __name__ == '__main__':
    main()
    print('test_static_aeroelastic_model passed')
