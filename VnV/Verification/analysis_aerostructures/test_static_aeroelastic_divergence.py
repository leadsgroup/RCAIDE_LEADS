# VnV/Verification/analysis_aerostructures/test_static_aeroelastic_divergence.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                                                      import Units, Data
from RCAIDE.Library.Methods.Geometry.Planform                                                   import wing_planform
from RCAIDE.Library.Mission.Common.Pre_Process                                                  import geometry_preprocess_routine
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.train_VLM_surrogates             import _converge_aeroelastic
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.generate_vortex_distribution     import generate_vortex_distribution
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.static_aeroelastic_model     import build_static_aeroelastic_model, solve_static_aeroelastic_model, divergence_dynamic_pressure, aerodynamic_conditions, VALIDATED_TIP_DEFLECTION_RATIO

import numpy as np

# a straight wing whose elastic axis (35% chord) is behind its aerodynamic centre diverges in torsion
MACH      = 0.3
ALPHA     = 0.02 * Units.degrees   # small, so the deflections stay small at every dynamic pressure
SEMI_SPAN = 10.0
CHORD     = 2.0

# ----------------------------------------------------------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    analyses = build_analyses()
    model    = build_static_aeroelastic_model(analyses.vehicle, analyses.aerodynamics.settings, analyses.aerostructures.settings,
                                              [MACH], number_of_stations=8)
    q_D      = divergence_dynamic_pressure(model, MACH, ALPHA)

    check_divergence_pressure_is_above_strip_theory(model, analyses, q_D)
    check_response_grows_toward_divergence(model, q_D)
    check_model_matches_the_coupled_iteration(model, analyses, q_D)
    check_solve_flags_dynamic_pressures_above_divergence(model, q_D)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def build_analyses():
    """One rectangular, unswept wing: 10 m semi-span, 2 m chord, spars at 10% and 60% chord, no inertial loads."""
    vehicle                = RCAIDE.Vehicle()
    vehicle.tag            = 'straight_wing'
    vehicle.reference_area = 2 * SEMI_SPAN * CHORD

    wing                         = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                     = 'main_wing'
    wing.spans.projected         = 2 * SEMI_SPAN
    wing.chords.root             = CHORD
    wing.chords.tip              = CHORD
    wing.chords.mean_aerodynamic = CHORD
    wing.taper                   = 1.0
    wing.areas.reference         = 2 * SEMI_SPAN * CHORD
    wing.aspect_ratio            = 2 * SEMI_SPAN / CHORD
    wing.thickness_to_chord      = 0.15
    wing.sweeps.quarter_chord    = 0.0
    wing.xz_plane_symmetric      = True
    for tag, span_location in [('root', 0.0), ('tip', 1.0)]:
        segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                   = tag
        segment.percent_span_location = span_location
        segment.root_chord_percent    = 1.0
        segment.thickness_to_chord    = 0.15
        segment.sweeps.quarter_chord  = 0.0
        segment.dihedral_outboard     = 0.0
        wing.append_segment(segment)
    wing_planform(wing)
    vehicle.append_component(wing)

    analyses         = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle
    geometry         = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.settings.compute_fuel_volume = False
    analyses.append(geometry)
    analyses.append(RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method())
    aerostructures = RCAIDE.Framework.Analyses.Aerostructures.Finite_Element_Analysis()
    aerostructures.settings.discretiation        = 100
    aerostructures.settings.load_factor          = 0.0
    aerostructures.settings.aeroelastic_coupling = 'coupled'
    analyses.append(aerostructures)
    analyses.append(RCAIDE.Framework.Analyses.Planets.Earth())
    analyses.append(RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976())
    geometry_preprocess_routine(analyses)
    return analyses


def tip_twist(model, q, coupled=True):
    return solve_static_aeroelastic_model(model, MACH, q, ALPHA, load_factor=0.0, coupled=coupled).main_wing.rotation[-1, 1]

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def check_divergence_pressure_is_above_strip_theory(model, analyses, q_D):
    # strip theory for a uniform wing: GJ theta'' + q e c a_0 theta = 0 with theta(0) = 0 and theta'(l) = 0 first
    # has a solution at q_D = (pi / 2l)^2 GJ / (e c a_0). It uses the two-dimensional lift slope a_0 = 2 pi, so it
    # is below the finite wing's
    wing      = analyses.vehicle.wings.main_wing
    GJ        = wing.structural.material.shear_modulus * model.wings.main_wing.beam.J[0]
    e         = (0.35 - 0.25) * CHORD
    q_D_strip = (np.pi / (2 * SEMI_SPAN))**2 * GJ / (e * CHORD * 2 * np.pi)
    assert q_D_strip < q_D < 2.0 * q_D_strip, (q_D, q_D_strip)
    return


def check_response_grows_toward_divergence(model, q_D):
    # with one dominant mode the coupled twist is the one-way twist times 1 / (1 - q / q_D)
    for ratio in (0.3, 0.5, 0.9):
        growth = tip_twist(model, ratio * q_D) / tip_twist(model, ratio * q_D, coupled=False)
        assert abs(growth * (1 - ratio) - 1.0) < 0.05, (ratio, growth)
    return


def check_model_matches_the_coupled_iteration(model, analyses, q_D):
    # the fixed-point iteration at half the divergence pressure, where it contracts by one half per iteration
    q                                                = 0.5 * q_D
    conditions                                       = aerodynamic_conditions(MACH, ALPHA)
    conditions.freestream.dynamic_pressure           = np.array([[q]])
    conditions.freestream.density                    = np.array([[1.225]])
    conditions.freestream.gravitational_acceleration = np.array([[9.81]])
    conditions.aerostructures                        = Data()
    conditions.control_surfaces                      = Data()
    conditions.weights                               = Data(components=Data(mass=Data()))
    jig_VD = generate_vortex_distribution(conditions, analyses.aerodynamics.settings, analyses.vehicle)
    _, structural, history = _converge_aeroelastic(conditions, analyses.aerodynamics.settings, analyses.vehicle, analyses.aerostructures, jig_VD, 1e-8, 100)

    direct = structural.main_wing.rotation[0, -1, 1]
    assert abs(tip_twist(model, q) - direct) < 0.02 * abs(direct), (tip_twist(model, q), direct)

    # its early steps, before round-off, shrink by q / q_D, which gives the divergence pressure independently of the model
    contraction = history[3] / history[2]
    assert abs(contraction - 0.5) < 0.02, contraction
    return


def check_solve_flags_dynamic_pressures_above_divergence(model, q_D):
    # the solve never raises; the determinant passes through zero at q_D and the flag follows it
    def solve(q):
        return solve_static_aeroelastic_model(model, MACH, q, ALPHA, load_factor=0.0)
    assert abs(solve(0.0).divergence_determinant - 1.0) < 1e-12
    assert solve(0.99 * q_D).divergence_determinant > 0.0
    assert solve(1.01 * q_D).divergence_determinant < 0.0
    assert solve(0.5 * q_D).within_validated_range
    assert not solve(1.01 * q_D).within_validated_range

    # below divergence, a deflection past the validated ratio is flagged too, and the solution is still returned
    large = solve_static_aeroelastic_model(model, MACH, 0.3 * q_D, 4.0 * Units.degrees, load_factor=0.0)
    assert large.divergence_determinant > 0.0
    assert large.main_wing.tip_deflection_ratio > VALIDATED_TIP_DEFLECTION_RATIO and not large.within_validated_range
    assert np.all(np.isfinite(large.main_wing.deflection))
    return


if __name__ == '__main__':
    main()
    print('test_static_aeroelastic_divergence passed')
