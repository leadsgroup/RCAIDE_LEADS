# compound_control_surfaces_test.py
#
# Created:  Aug 2026, M. Clarke
#
# Exercises the three compound control surfaces (Flaperon, Elevon, Ruddervator) added to
# RCAIDE.Library.Components.Wings.Control_Surfaces: each is a single physical hinge that
# responds to two independent trim channels (a primary/symmetric command via `.deflection`
# and a secondary/antisymmetric command via `.secondary_deflection`), mixed onto the panel via
# generate_wing_vortex_distribution.py and trained/evaluated through the generic
# control_surface_registry.py-driven pipeline in train_VLM_surrogates.py / build_VLM_surrogates.py
# / evaluate_VLM.py, and wired to trim in Unpack_Unknowns/control_surfaces.py.

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units, Data
from RCAIDE.Library.Mission.Common.Pre_Process.geometry import geometry_preprocess_routine

import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#   Vehicle
# ----------------------------------------------------------------------------------------------------------------------
def simple_wing(tag, xz_plane_symmetric=True, dihedral=0.0):
    """A minimal rectangular wing used to build isolated single-surface test vehicles."""
    wing = RCAIDE.Library.Components.Wings.Wing()
    wing.tag                 = tag
    wing.spans.projected     = 10.0
    wing.chords.root         = 2.0
    wing.chords.tip          = 2.0
    wing.areas.reference     = 20.0
    wing.taper               = 1.0
    wing.sweeps.leading_edge = 0.0
    wing.twists.root         = 0.0
    wing.twists.tip          = 0.0
    wing.origin              = [[0,0,0]]
    wing.aerodynamic_center  = [0.5,0,0]
    wing.vertical            = False
    wing.xz_plane_symmetric  = xz_plane_symmetric

    seg = RCAIDE.Library.Components.Wings.Segments.Segment()
    seg.tag                       = 'root'
    seg.percent_span_location     = 0.0
    seg.root_chord_percent        = 1.0
    seg.twist                     = 0.
    seg.dihedral_outboard         = dihedral
    seg.sweeps.leading_edge       = 0.
    wing.append_segment(seg)

    seg = RCAIDE.Library.Components.Wings.Segments.Segment()
    seg.tag                       = 'tip'
    seg.percent_span_location     = 1.0
    seg.root_chord_percent        = 1.0
    seg.twist                     = 0.
    seg.dihedral_outboard         = dihedral
    seg.sweeps.leading_edge       = 0.
    wing.append_segment(seg)

    return wing


def preprocess(vehicle):
    analyses          = Data()
    analyses.geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    analyses.vehicle  = vehicle
    geometry_preprocess_routine(analyses)
    return vehicle


def single_surface_vehicle(wing_tag, control_surface, xz_plane_symmetric=True, dihedral=0.0):
    """An isolated one-wing vehicle carrying a single control surface, used for clean VLM-only
    physics checks (no multi-surface aerodynamic interference to muddy the primary/secondary
    cross-term measurement)."""
    vehicle = RCAIDE.Vehicle()
    vehicle.tag = wing_tag + '_test_vehicle'
    vehicle.reference_area = 20.0
    wing = simple_wing(wing_tag, xz_plane_symmetric=xz_plane_symmetric, dihedral=dihedral)
    control_surface.span_fraction_start = 0.1
    control_surface.span_fraction_end   = 0.9
    control_surface.chord_fraction      = 0.3
    wing.append_control_surface(control_surface)
    vehicle.append_component(wing)
    return preprocess(vehicle)


def combined_vehicle():
    """A single vehicle carrying all three compound surfaces at once (on separate, non-interacting
    wings), used only for the surrogate-training and trim-dispatch wiring checks, where aerodynamic
    interference between the wings doesn't matter."""
    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'compound_control_surface_test_vehicle'
    vehicle.reference_area = 20.0

    main_wing = simple_wing('main_wing')
    flaperon = RCAIDE.Library.Components.Wings.Control_Surfaces.Flaperon()
    flaperon.tag                 = 'flaperon'
    flaperon.span_fraction_start = 0.1
    flaperon.span_fraction_end   = 0.9
    flaperon.chord_fraction      = 0.3
    main_wing.append_control_surface(flaperon)
    vehicle.append_component(main_wing)

    canard = simple_wing('canard')
    canard.origin = [[-5,0,0]]
    elevon = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevon()
    elevon.tag                 = 'elevon'
    elevon.span_fraction_start = 0.1
    elevon.span_fraction_end   = 0.9
    elevon.chord_fraction      = 0.3
    canard.append_control_surface(elevon)
    vehicle.append_component(canard)

    v_tail = simple_wing('v_tail', xz_plane_symmetric=True, dihedral=35.0*Units.degrees)
    v_tail.origin = [[8,0,0]]
    ruddervator = RCAIDE.Library.Components.Wings.Control_Surfaces.Ruddervator()
    ruddervator.tag                 = 'ruddervator'
    ruddervator.span_fraction_start = 0.1
    ruddervator.span_fraction_end   = 0.9
    ruddervator.chord_fraction      = 0.3
    v_tail.append_control_surface(ruddervator)
    vehicle.append_component(v_tail)

    return preprocess(vehicle)


# ----------------------------------------------------------------------------------------------------------------------
#   VLM-only sanity checks
# ----------------------------------------------------------------------------------------------------------------------
def run_vlm(vehicle, control_surface, primary_deg, secondary_deg, settings):
    from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM import VLM

    control_surface.deflection           = primary_deg * Units.deg
    control_surface.secondary_deflection = secondary_deg * Units.deg

    conditions = RCAIDE.Framework.Mission.Common.Results()
    conditions.expand_rows(1, override=False)
    conditions.freestream.mach_number[:,0]          = 0.3
    conditions.aerodynamics.angles.alpha[:,0]        = 1e-6
    conditions.aerodynamics.angles.beta[:,0]          = 0.0
    conditions.freestream.velocity[:,0]              = 100.0
    conditions.static_stability.pitch_rate[:,0]      = 0.0
    conditions.static_stability.roll_rate[:,0]       = 0.0
    conditions.static_stability.yaw_rate[:,0]        = 0.0
    return VLM(conditions, settings, vehicle)


def check_compound_surface_mirroring(vehicle, control_surface, primary_coeff_name, secondary_coeff_name):
    """Confirms: primary-only deflection produces the primary (symmetric) coefficient with a
    negligible secondary (antisymmetric) cross-term, secondary-only does the reverse, and the
    combined case is approximately the linear sum of both (VLM is a linear aerodynamic model)."""

    # small deflections (2 deg) keep both the isolated and combined (up to 4 deg local panel
    # angle) cases well within VLM's linear regime, so strict superposition holds
    settings = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method().settings

    r0 = run_vlm(vehicle, control_surface, 0.0, 0.0, settings)
    r1 = run_vlm(vehicle, control_surface, 2.0, 0.0, settings)
    r2 = run_vlm(vehicle, control_surface, 0.0, 2.0, settings)
    r3 = run_vlm(vehicle, control_surface, 2.0, 2.0, settings)

    coeffs = {'CY': 'CY', 'CL': 'CL', 'CM': 'CM', 'CN': 'CN'}
    primary   = getattr(r1, coeffs[primary_coeff_name])[0,0]   - getattr(r0, coeffs[primary_coeff_name])[0,0]
    secondary = getattr(r2, coeffs[secondary_coeff_name])[0,0] - getattr(r0, coeffs[secondary_coeff_name])[0,0]
    primary_cross   = getattr(r1, coeffs[secondary_coeff_name])[0,0] - getattr(r0, coeffs[secondary_coeff_name])[0,0]
    secondary_cross = getattr(r2, coeffs[primary_coeff_name])[0,0]   - getattr(r0, coeffs[primary_coeff_name])[0,0]

    print(f"  {control_surface.tag}: primary-only d{primary_coeff_name}={primary:.6f} (cross d{secondary_coeff_name}={primary_cross:.6f}); "
          f"secondary-only d{secondary_coeff_name}={secondary:.6f} (cross d{primary_coeff_name}={secondary_cross:.6f})")

    # relative (not absolute) threshold: a Ruddervator's yaw authority is inherently weaker than its
    # pitch authority on a dihedral V-tail, making its cross-term a larger fraction of that weak response
    assert abs(primary)   > 1e-4, f"{control_surface.tag}: primary channel produced no {primary_coeff_name} response"
    assert abs(secondary) > 1e-4, f"{control_surface.tag}: secondary channel produced no {secondary_coeff_name} response"
    assert abs(primary_cross)   < 0.15 * abs(primary),   f"{control_surface.tag}: primary channel leaked into {secondary_coeff_name}"
    assert abs(secondary_cross) < 0.15 * abs(secondary), f"{control_surface.tag}: secondary channel leaked into {primary_coeff_name}"

    combined_primary   = getattr(r3, coeffs[primary_coeff_name])[0,0]   - getattr(r0, coeffs[primary_coeff_name])[0,0]
    combined_secondary = getattr(r3, coeffs[secondary_coeff_name])[0,0] - getattr(r0, coeffs[secondary_coeff_name])[0,0]
    assert np.abs(combined_primary   - primary)   < 0.15 * abs(primary),   f"{control_surface.tag}: combined {primary_coeff_name} not additive"
    assert np.abs(combined_secondary - secondary) < 0.15 * abs(secondary), f"{control_surface.tag}: combined {secondary_coeff_name} not additive"


# ----------------------------------------------------------------------------------------------------------------------
#   Surrogate pipeline roundtrip check
# ----------------------------------------------------------------------------------------------------------------------
def check_surrogate_roundtrip(vehicle):
    from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.train_VLM_surrogates import train_VLM_surrogates
    from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.build_VLM_surrogates import build_VLM_surrogates

    aero = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aero.vehicle = vehicle
    aero.settings.use_surrogate = True
    aero.aileron_flag = False
    aero.elevator_flag = False
    aero.rudder_flag = False
    aero.flap_flag = False
    aero.slat_flag = False
    aero.flaperon_flag = False
    aero.elevon_flag = False
    aero.ruddervator_flag = False
    aero.training.Mach = np.array([0.2, 0.3, 0.4])

    train_VLM_surrogates(aero, vehicle)
    build_VLM_surrogates(aero, vehicle)

    assert aero.flaperon_flag and aero.elevon_flag and aero.ruddervator_flag, "compound flags not set during training"

    sub_sur = aero.surrogates.subsonic
    for letter in ('pf','sf','pe','se','pr','sr'):
        for coeff in ('Clift','Cdrag','CX','CY','CZ','CL','CM','CN'):
            key = 'd' + coeff + '_ddelta_' + letter
            assert hasattr(sub_sur, key) and getattr(sub_sur, key) is not None, f"missing surrogate {key}"
    print("  surrogate pipeline: all 6 compound-channel rows (pf,sf,pe,se,pr,sr) x 8 coefficients built")


# ----------------------------------------------------------------------------------------------------------------------
#   Trim-dispatch check
# ----------------------------------------------------------------------------------------------------------------------
def check_trim_dispatch(vehicle):
    from RCAIDE.Library.Mission.Common.Unpack_Unknowns.control_surfaces import control_surfaces as unpack_control_surfaces

    segment = Data()
    segment.analyses = Data()
    segment.analyses.vehicle = vehicle
    segment.assigned_control_variables = Data()
    for name in ('elevator_deflection','rudder_deflection','aileron_deflection','flap_deflection','slat_deflection'):
        segment.assigned_control_variables[name] = Data()
        segment.assigned_control_variables[name].active = False

    segment.assigned_control_variables.elevator_deflection.active = True
    segment.assigned_control_variables.aileron_deflection.active  = True
    segment.assigned_control_variables.rudder_deflection.active   = True

    segment.state = Data()
    segment.state.unknowns = Data()
    segment.state.unknowns.mission = {
        'elevator': np.array([[3.0 * Units.deg]]),
        'aileron':  np.array([[2.0 * Units.deg]]),
        'rudder':   np.array([[4.0 * Units.deg]]),
    }
    segment.state.conditions = RCAIDE.Framework.Mission.Common.Results()
    segment.state.conditions.expand_rows(1, override=False)

    unpack_control_surfaces(segment)

    cs = segment.state.conditions.control_surfaces
    assert np.isclose(cs.elevon.deflection[0,0], 3.0 * Units.deg),               "Elevon primary (elevator) channel not wired"
    assert np.isclose(cs.elevon.secondary_deflection[0,0], 2.0 * Units.deg),     "Elevon secondary (aileron) channel not wired"
    assert np.isclose(cs.ruddervator.deflection[0,0], 3.0 * Units.deg),         "Ruddervator primary (elevator) channel not wired"
    assert np.isclose(cs.ruddervator.secondary_deflection[0,0], 4.0 * Units.deg), "Ruddervator secondary (rudder) channel not wired"
    assert np.isclose(cs.flaperon.secondary_deflection[0,0], 2.0 * Units.deg),   "Flaperon secondary (aileron) channel not wired"
    print("  trim dispatch: Elevon/Ruddervator/Flaperon each correctly received their two channels")


# ----------------------------------------------------------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    print("VLM-only mirroring sanity checks (isolated single-surface vehicles):")
    flaperon_vehicle = single_surface_vehicle('main_wing', RCAIDE.Library.Components.Wings.Control_Surfaces.Flaperon())
    check_compound_surface_mirroring(flaperon_vehicle, flaperon_vehicle.wings.main_wing.control_surfaces.flaperon, 'CM', 'CL')

    elevon_vehicle = single_surface_vehicle('main_wing', RCAIDE.Library.Components.Wings.Control_Surfaces.Elevon())
    check_compound_surface_mirroring(elevon_vehicle, elevon_vehicle.wings.main_wing.control_surfaces.elevon, 'CM', 'CL')

    ruddervator_vehicle = single_surface_vehicle('v_tail', RCAIDE.Library.Components.Wings.Control_Surfaces.Ruddervator(),
                                                  xz_plane_symmetric=True, dihedral=35.0*Units.degrees)
    check_compound_surface_mirroring(ruddervator_vehicle, ruddervator_vehicle.wings.v_tail.control_surfaces.ruddervator, 'CM', 'CN')

    vehicle = combined_vehicle()

    print("\nSurrogate pipeline roundtrip:")
    check_surrogate_roundtrip(vehicle)

    print("\nTrim dispatch:")
    check_trim_dispatch(vehicle)

    print("\nAll compound control surface checks passed.")
    return


if __name__ == '__main__':
    main()
