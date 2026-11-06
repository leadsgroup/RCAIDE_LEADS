# VnV/Verification/analysis_aerodynamics/test_VLM_section_correction.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                         import Units, Data
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method     import VLM
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM import compute_strip_sweep, compute_taper_lift_factor, compute_conical_surface_cp, compute_normal_mach
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.generate_vortex_distribution import generate_vortex_distribution
from RCAIDE.Library.Methods.Geometry.Planform                      import wing_planform
import numpy as np
import pytest

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
HALF_CHORD_SWEEP = 22.9 * Units.degrees


def build_vehicle():
    # ONERA M6 planform
    taper                  = 0.562
    chord                  = 1.5 * 0.64607 * (1.0 + taper) / (1.0 + taper + taper**2)
    wing                   = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag               = "main_wing"
    wing.symmetric         = True
    wing.spans.projected   = 2.0 * 1.1963
    wing.chords.root       = chord

    for tag, location, chord_percent in (("Root", 0.0, 1.0), ("Tip", 1.0, taper)):
        segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                   = tag
        segment.percent_span_location = location
        segment.root_chord_percent    = chord_percent
        segment.twist                 = 0.0
        segment.dihedral_outboard     = 0.0
        segment.sweeps.quarter_chord  = 26.390665 * Units.degrees
        segment.thickness_to_chord    = 0.10
        wing.append_segment(segment)
    wing_planform(wing)

    vehicle                = RCAIDE.Vehicle()
    vehicle.append_component(wing)
    vehicle.reference_area = wing.areas.reference
    vehicle.reference_span = wing.spans.projected
    vehicle.reference_chord = wing.chords.mean_aerodynamic
    vehicle.mass_properties.center_of_gravity = [[0.25 * chord, 0.0, 0.0]]
    return vehicle


def build_conditions(alpha_deg, mach=0.84):
    alpha_deg  = np.atleast_1d(alpha_deg)[:, None]
    conditions = RCAIDE.Framework.Mission.Common.Results()
    conditions.freestream.mach_number      = np.full_like(alpha_deg, mach, dtype=float)
    conditions.freestream.velocity         = conditions.freestream.mach_number * 340.0
    conditions.aerodynamics.angles.alpha   = alpha_deg * Units.degrees
    conditions.aerodynamics.angles.beta    = np.zeros_like(alpha_deg, dtype=float)
    for name in ("pitch_rate", "roll_rate", "yaw_rate"):
        conditions.static_stability[name]  = np.zeros_like(alpha_deg, dtype=float)
    return conditions


def build_settings(fidelity, model=None):
    settings = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method().settings
    settings.number_of_spanwise_vortices   = 8
    settings.number_of_chordwise_vortices  = 4
    settings.spanwise_cosine_spacing       = False
    settings.floating_point_precision      = np.float64
    settings.propeller_wake_model          = False
    settings.discretize_control_surfaces   = False
    settings.crossflow_fidelity            = fidelity
    settings.section_lift_model            = model
    return settings


def thin_airfoil(alpha_2d, mach_2d, thickness_scale):
    return Data(cl = 2 * np.pi / np.sqrt(1 - mach_2d**2) * alpha_2d)


def transonic_cl(alpha_2d):
    # Lift curve with a steep rise and a plateau, the shape that makes plain relaxation fail
    knots = np.radians([0.0, 1.0, 1.6, 2.0, 2.4, 5.0])
    cls   = np.array([0.0, 0.25, 0.50, 0.97, 1.39, 1.54])
    return np.sign(alpha_2d) * np.interp(np.abs(alpha_2d), knots, cls)


def transonic_section(alpha_2d, mach_2d, thickness_scale):
    return Data(cl = transonic_cl(alpha_2d))


def thin_airfoil_with_pressure(alpha_2d, mach_2d, thickness_scale):
    # Flat-plate chordwise load for the 2.75D factor
    n_cases, n_strips = alpha_2d.shape
    x        = np.linspace(0.005, 1.0, 200)
    delta_cp = np.sqrt((1 - x) / x)
    section  = thin_airfoil(alpha_2d, mach_2d, thickness_scale)
    section.x        = [[x for strip in range(n_strips)] for case in range(n_cases)]
    section.delta_cp = [[delta_cp for strip in range(n_strips)] for case in range(n_cases)]
    return section

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def test_strip_half_chord_sweep():
    VD = generate_vortex_distribution(build_conditions(3.0), build_settings('1D'), build_vehicle())
    assert np.allclose(compute_strip_sweep(VD, 0.5), HALF_CHORD_SWEEP, atol=1e-6)
    assert np.allclose(compute_strip_sweep(VD, 0.25), 26.390665 * Units.degrees, atol=1e-6)
    assert np.allclose(compute_strip_sweep(VD, 0.), 29.682533 * Units.degrees, atol=1e-6)


def test_thin_airfoil_model_needs_no_correction():
    baseline = VLM(build_conditions(3.0), build_settings('1D'), build_vehicle())
    results  = VLM(build_conditions(3.0), build_settings('2D', thin_airfoil), build_vehicle())
    assert results.section_iterations == 0
    assert np.allclose(results.sectional_CLift, baseline.sectional_CLift, rtol=1e-10, atol=1e-10)
    assert np.all(results.sectional_delta_alpha == 0)


@pytest.mark.parametrize("alpha_deg", [3.0, 6.0, 8.0])
def test_converges_on_transonic_lift_curve(alpha_deg):
    settings = build_settings('2D', transonic_section)
    results  = VLM(build_conditions(alpha_deg), settings, build_vehicle())
    cl       = results.sectional_CLift
    a        = 2 * np.pi / np.sqrt(1 - (0.84 * np.cos(HALF_CHORD_SWEEP))**2)
    alpha_2d = cl / a - results.sectional_delta_alpha
    target   = transonic_cl(alpha_2d)
    assert results.section_iterations > 0
    assert np.max(np.abs(target - cl)) < 2 * settings.section_correction_tolerance


def test_multiple_cases_match_single_cases():
    alphas   = [3.0, 6.0]
    together = VLM(build_conditions(alphas), build_settings('2D', transonic_section), build_vehicle())
    for i, alpha_deg in enumerate(alphas):
        single = VLM(build_conditions(alpha_deg), build_settings('2D', transonic_section), build_vehicle())
        assert together.CLift[i, 0] == pytest.approx(single.CLift.item(), abs=1e-3)


def test_missing_model_raises():
    with pytest.raises(ValueError):
        VLM(build_conditions(3.0), build_settings('2D'), build_vehicle())


def test_nonconvergence_warns_and_returns_best_iterate():
    settings = build_settings('2D', transonic_section)
    settings.section_correction_max_iterations = 1
    with pytest.warns(UserWarning):
        results = VLM(build_conditions(6.0), settings, build_vehicle())
    assert results.section_converged is False
    assert results.section_residual > settings.section_correction_tolerance


def test_section_model_nan_raises():
    def nan_section(alpha_2d, mach_2d, thickness_scale):
        return Data(cl = np.full_like(alpha_2d, np.nan))
    with pytest.raises(RuntimeError):
        VLM(build_conditions(3.0), build_settings('2D', nan_section), build_vehicle())


def test_2p5D_thin_airfoil_needs_no_correction():
    baseline = VLM(build_conditions(3.0), build_settings('1D'), build_vehicle())
    results  = VLM(build_conditions(3.0), build_settings('2.5D', thin_airfoil), build_vehicle())
    assert results.section_iterations == 0
    assert np.allclose(results.sectional_CLift, baseline.sectional_CLift, rtol=1e-10, atol=1e-10)


def test_2p75D_converges_and_lift_differs_from_2p5D():
    yawed   = VLM(build_conditions(3.0), build_settings('2.5D', transonic_section), build_vehicle())
    tapered = VLM(build_conditions(3.0), build_settings('2.75D', thin_airfoil_with_pressure), build_vehicle())
    assert tapered.section_iterations > 0
    assert tapered.CLift.item() != pytest.approx(yawed.CLift.item(), abs=1e-6)


def test_taper_factor_is_one_without_taper():
    x        = np.linspace(0.005, 1.0, 200)
    delta_cp = np.sqrt((1 - x) / x)
    tan_mean = np.tan(30 * Units.degrees)
    assert compute_taper_lift_factor(x, delta_cp, 0.84, 30 * Units.degrees, tan_mean, tan_mean) == pytest.approx(1.0)


def test_taper_factor_matches_lock_equation_3():
    # F < 1 where local sweep exceeds the mean sweep, F = 1 aft of it (Lock and Bridgewater 1967)
    x        = np.array([0.0, 0.25, 1.0])
    delta_cp = np.array([1.0, 1.0, 1.0])
    mach     = 0.8
    tan_le   = np.tan(35 * Units.degrees)
    tan_te   = np.tan(15 * Units.degrees)
    tan_mean = tan_le - 0.25 * (tan_le - tan_te)
    sweep    = np.arctan(tan_mean)
    F_le     = ((1 + 0.2 * mach**2 * np.cos(35 * Units.degrees)**2) / (1 + 0.2 * mach**2 * np.cos(sweep)**2))**3.5
    expected = (0.25 * 0.5 * (F_le + 1.0) + 0.75) / 1.0
    assert F_le < 1.0
    assert compute_taper_lift_factor(x, delta_cp, mach, sweep, tan_le, tan_te) == pytest.approx(expected)


def test_conical_surface_cp_is_zero_in_undisturbed_flow():
    # A flat conical plate at the local normal Mach number of the freestream must recover Cp = 0
    x      = np.linspace(0.0, 1.0, 401)
    tan_le = np.tan(30 * Units.degrees)
    tan_te = np.tan(15 * Units.degrees)
    mach   = 0.8
    z_0    = 1 / (tan_le - tan_te)
    x_0    = z_0 * tan_le
    M_n    = mach * z_0 / np.sqrt((x_0 - x)**2 + z_0**2)
    cp     = compute_conical_surface_cp(x, np.zeros_like(x), M_n, mach, tan_le, tan_te)
    assert np.max(np.abs(cp)) < 1e-6


def test_conical_surface_cp_recovers_yawed_wing_without_taper():
    # With almost no taper, Lock's method reduces to Cp = Cp* cos^2(sweep)
    x      = np.linspace(0.0, 1.0, 401)
    sweep  = 30 * Units.degrees
    mach   = 0.8
    cos2   = np.cos(sweep)**2
    cp_2d  = -0.5 * np.sin(np.pi * x)
    M_n    = compute_normal_mach(cp_2d, mach * np.cos(sweep))
    cp     = compute_conical_surface_cp(x, np.zeros_like(x), M_n, mach, np.tan(sweep) + 1e-5, np.tan(sweep) - 1e-5)
    assert np.allclose(cp, cp_2d * cos2, atol=1e-4)


def test_normal_mach_matches_freestream_at_zero_cp():
    assert compute_normal_mach(np.array([0.0]), 0.7)[0] == pytest.approx(0.7)


@pytest.mark.parametrize("fidelity", ['Karman-Tsien', 'full_3D_correction'])
def test_unbuilt_fidelities_raise(fidelity):
    with pytest.raises(NotImplementedError):
        VLM(build_conditions(3.0), build_settings(fidelity, thin_airfoil), build_vehicle())
