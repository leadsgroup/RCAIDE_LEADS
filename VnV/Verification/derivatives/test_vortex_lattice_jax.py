# VnV/Verification/derivatives/test_vortex_lattice_jax.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys
import numpy as np

import RCAIDE
from RCAIDE.Framework.Core import Data, set_at
from RCAIDE.Library.Mission.Common.Pre_Process.geometry import geometry_preprocess_routine
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Geometry.Planform import wing_planform
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method import VLM, generate_vortex_distribution, compute_wing_induced_velocity
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method import wing_lattice_layout
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM import LATTICE_FIELDS
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.generate_vortex_distribution import generate_aircraft_vortex_distribution, assemble_vortex_distribution
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.postprocess_vortex_distribution import postprocess_vortex_distribution, PANEL_FIELDS
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.train_VLM_surrogates import control_surface_conditions

# lattice coordinates the influence matrix is computed from
COORDINATES = ('XAH', 'YAH', 'ZAH', 'XBH', 'YBH', 'ZBH', 'XC', 'YC', 'ZC')

# lattice fields the force and moment coefficients do not depend on: the aft corners of the panels, which only
# place the trailing edge, and the wing areas, which only scale the lift coefficient of each wing
NOT_IN_THE_COEFFICIENTS = ('XA2', 'XB2', 'YA2', 'YB2', 'ZA2', 'ZB2', 'wing_areas')

# what the generator of the lattice produces: the panel corners, the bound vortex and control points, the points
# where the loads act and the corner nodes; chords, slopes, normals and incidences are derived from them
GENERATED_FIELDS = PANEL_FIELDS + ('XCH', 'YCH', 'ZCH', 'X', 'Y', 'Z')

# generated fields the coefficients do not depend on (the longitudinal position of the nodes is only plotted)
GENERATED_NOT_IN_THE_COEFFICIENTS = ('XAC', 'XBC', 'X')

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # The vortex lattice method is written once for NumPy and JAX, so that the aerodynamic coefficients can be
    # differentiated with respect to the geometry. Each stage is checked on a conventional transport (wing, tails) and
    # on a blended wing body: JAX values against NumPy, the compiled function, the forward-mode derivative against
    # central differences of the NumPy path, forward against reverse mode, and that every input reaches an output.
    try:
        import jax
    except ImportError:
        print("JAX is not installed (pip install RCAIDE[ad]); JAX checks skipped")
        return
    jax.config.update("jax_enable_x64", True)
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Vehicles")))

    for name in ('Embraer_190', 'BWB'):
        vehicle, settings, lattice = coarse_lattice(name)
        check_influence_matrix(name, lattice)
        check_solve(name, vehicle, settings, lattice)
        check_derived_lattice(name, vehicle, settings)
        check_wing_lattice(name, vehicle, settings)
        for wing in vehicle.wings:
            check_wing_planform(name, vehicle, settings, wing)
        check_control_surfaces(name, vehicle, settings)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_influence_matrix(name, lattice):
    from RCAIDE.Library.Methods.Utilities.derivative_check import check_derivatives
    mach  = np.array([[0.6]])
    phi   = np.arctan((lattice.ZBC - lattice.ZAC) / (lattice.YBC - lattice.YAC))
    delta = np.arctan((lattice.ZC - lattice.ZCH) / (lattice.XC - lattice.XCH))

    # The two ends of a horseshoe vortex on a vertical surface have the same lateral coordinate, and which end is
    # taken first follows from which is larger. Moving one end sideways alone flips that order, a jump no change of
    # the geometry produces, so the lateral coordinates of those panels are held and the others are inputs.
    vertical = np.isclose(lattice.YAH, lattice.YBH)[0]
    free     = np.flatnonzero(~vertical)

    def influence(*arrays):
        moved = Data()
        for key in lattice.keys():
            moved[key] = lattice[key]
        for key, value in zip(COORDINATES, arrays):
            if key in ('YAH', 'YBH'):
                value = set_at(np.array(lattice[key], dtype=float), (slice(None), free), value)
            moved[key] = value
        A, s, RFLAG, EW = compute_wing_induced_velocity(moved, mach, compute_EW=True, delta=arrays[-2], phi=arrays[-1])
        return A, s, EW

    inputs  = tuple(np.array(lattice[key], dtype=float)[:, free] if key in ('YAH', 'YBH') else np.array(lattice[key], dtype=float)
                    for key in COORDINATES) + (np.array(delta, dtype=float), np.array(phi, dtype=float))
    report  = check_derivatives(influence, inputs, steps=(1e-5, 1e-6, 1e-7), n_directions=2)
    reached = [float(np.max(np.abs(sensitivity))) for sensitivity in report.input_sensitivity]
    print(f"  {name} influence matrix, {len(vertical)} panels ({int(np.sum(vertical))} on vertical surfaces): values {report.value_error:.1e}, "
          f"derivatives {report.jvp_error:.1e}, reverse {report.vjp_consistency_error:.1e}, weakest input {min(reached):.1e}")
    assert report.passed, f"{name}: influence matrix JAX check failed, {report}"
    assert min(reached) > 0, f"{name}: an input does not reach the influence matrix, {dict(zip(COORDINATES + ('delta', 'phi'), reached))}"
    return

def check_solve(name, vehicle, settings, lattice):
    # lift, induced drag, the three moments, side force and the vortex strengths, with sideslip and body rates,
    # differentiated with respect to every field of the lattice the solve reads
    from RCAIDE.Library.Methods.Utilities.derivative_check import check_derivatives
    free = free_entries(lattice)

    def solve(*arrays):
        moved = Data()
        for key in lattice.keys():
            moved[key] = lattice[key]
        for key, value in zip(LATTICE_FIELDS, arrays):
            if key in free:
                value = set_at(np.array(lattice[key], dtype=float), (slice(None), free[key]), value)
            moved[key] = value
        results = VLM(flow_conditions(), settings, vehicle, VD=moved)
        return results.CLift, results.CDrag_induced, results.CM, results.CY, results.CL, results.CN, results.gamma

    inputs  = tuple(np.array(lattice[key], dtype=float)[:, free[key]] if key in free else np.array(lattice[key], dtype=float)
                    for key in LATTICE_FIELDS)
    report  = check_derivatives(solve, inputs, steps=(1e-5, 1e-6, 1e-7), n_directions=2)
    reached = {key: float(np.max(np.abs(sensitivity))) for key, sensitivity in zip(LATTICE_FIELDS, report.input_sensitivity)}
    missing = sorted(key for key, value in reached.items() if value == 0 and key not in NOT_IN_THE_COEFFICIENTS)
    print(f"  {name} solve and forces, {len(LATTICE_FIELDS)} lattice fields: values {report.value_error:.1e}, derivatives {report.jvp_error:.1e}, "
          f"reverse {report.vjp_consistency_error:.1e}")
    assert report.passed, f"{name}: solve JAX check failed, {report}"
    assert len(missing) == 0, f"{name}: lattice fields that do not reach the coefficients: {missing}"
    return

def check_derived_lattice(name, vehicle, settings):
    # the same coefficients, differentiated with respect to what the generator of the lattice produces, through the
    # chords, slopes, normals and incidences derived from it
    from RCAIDE.Library.Methods.Utilities.derivative_check import check_derivatives
    generated = generate_aircraft_vortex_distribution(vehicle, settings)
    free      = free_entries(assemble_vortex_distribution(generated, 1))

    def coefficients(*arrays):
        moved = Data()
        for key in generated.keys():
            moved[key] = generated[key]
        for key, value in zip(GENERATED_FIELDS, arrays):
            if key in free:
                value = set_at(np.array(generated[key], dtype=float), free[key], value)
            moved[key] = value
        lattice = assemble_vortex_distribution(postprocess_vortex_distribution(moved, settings), 1)
        results = VLM(flow_conditions(), settings, vehicle, VD=lattice)
        return results.CLift, results.CDrag_induced, results.CM, results.CY, results.CL, results.CN

    inputs  = tuple(np.array(generated[key], dtype=float)[free[key]] if key in free else np.array(generated[key], dtype=float)
                    for key in GENERATED_FIELDS)
    report  = check_derivatives(coefficients, inputs, steps=(1e-5, 1e-6, 1e-7), n_directions=2)
    reached = {key: float(np.max(np.abs(sensitivity))) for key, sensitivity in zip(GENERATED_FIELDS, report.input_sensitivity)}
    missing = sorted(key for key, value in reached.items() if value == 0 and key not in GENERATED_NOT_IN_THE_COEFFICIENTS)
    print(f"  {name} derived lattice, {len(GENERATED_FIELDS)} generated fields: values {report.value_error:.1e}, derivatives {report.jvp_error:.1e}, "
          f"reverse {report.vjp_consistency_error:.1e}")
    assert report.passed, f"{name}: derived lattice JAX check failed, {report}"
    assert len(missing) == 0, f"{name}: generated fields that do not reach the coefficients: {missing}"
    return

def check_wing_lattice(name, vehicle, settings):
    # the same coefficients, differentiated with respect to the numbers of the main wing its lattice is generated
    # from: span, root chord, and the chord fraction, twist, dihedral, leading-edge sweep and origin of each segment
    from RCAIDE.Library.Methods.Utilities.derivative_check import check_derivatives
    wing     = vehicle.wings.main_wing
    segments = list(wing.segments.values())
    layouts  = {surface.tag: wing_lattice_layout(surface, settings.number_of_chordwise_vortices, settings.number_of_spanwise_vortices,
                                                 settings.spanwise_cosine_spacing) for surface in vehicle.wings}
    names    = ('span', 'root chord', 'chord fractions', 'twists', 'dihedrals', 'leading-edge sweeps', 'origins x', 'origins z')

    def read():
        return (np.array([wing.spans.projected], dtype=float), np.array([wing.chords.root], dtype=float),
                np.array([segment.root_chord_percent for segment in segments], dtype=float),
                np.array([segment.twist for segment in segments], dtype=float),
                np.array([segment.dihedral_outboard for segment in segments], dtype=float),
                np.array([segment.sweeps.leading_edge for segment in segments], dtype=float),
                np.array([segment.origin[0][0] for segment in segments], dtype=float),
                np.array([segment.origin[0][2] for segment in segments], dtype=float))

    def write(span, root_chord, chord_fractions, twists, dihedrals, sweeps, origins_x, origins_z):
        wing.spans.projected = span[0]
        wing.chords.root     = root_chord[0]
        for i, segment in enumerate(segments):
            segment.root_chord_percent  = chord_fractions[i]
            segment.twist               = twists[i]
            segment.dihedral_outboard   = dihedrals[i]
            segment.sweeps.leading_edge = sweeps[i]
            segment.origin              = [[origins_x[i], segment.origin[0][1], origins_z[i]]]

    original = read()

    def coefficients(*arrays):
        try:
            write(*arrays)
            lattice = generate_vortex_distribution(flow_conditions(), settings, vehicle, layouts)
            results = VLM(flow_conditions(), settings, vehicle, VD=lattice)
            return results.CLift, results.CDrag_induced, results.CM, results.CY, results.CL, results.CN
        finally:
            write(*original)

    report  = check_derivatives(coefficients, original, steps=(1e-5, 1e-6, 1e-7), n_directions=2)
    reached = {key: float(np.max(np.abs(sensitivity))) for key, sensitivity in zip(names, report.input_sensitivity)}
    print(f"  {name} wing lattice, {sum(value.size for value in original)} numbers of the main wing: values {report.value_error:.1e}, "
          f"derivatives {report.jvp_error:.1e}, reverse {report.vjp_consistency_error:.1e}")
    assert report.passed, f"{name}: wing lattice JAX check failed, {report}"
    assert min(reached.values()) > 0, f"{name}: numbers of the wing that do not reach the coefficients: {reached}"
    return


def check_wing_planform(name, vehicle, settings, wing):
    # the planform values and the coefficients, differentiated with respect to the numbers that define one wing. The
    # segment origins and leading-edge sweeps of the lattice follow from them in wing_planform.
    from RCAIDE.Library.Methods.Utilities.derivative_check import check_derivatives
    layouts = {surface.tag: wing_lattice_layout(surface, settings.number_of_chordwise_vortices, settings.number_of_spanwise_vortices,
                                                settings.spanwise_cosine_spacing) for surface in vehicle.wings}
    names, read, write = defining_numbers(wing)
    original = read()

    def planform_and_coefficients(*arrays):
        try:
            write(*arrays)
            wing_planform(wing)
            lattice = generate_vortex_distribution(flow_conditions(), settings, vehicle, layouts)
            results = VLM(flow_conditions(), settings, vehicle, VD=lattice)
            return (wing.areas.reference, wing.areas.wetted, wing.chords.mean_aerodynamic, wing.aspect_ratio, wing.spans.total,
                    wing.aerodynamic_center, wing.LEMAC, results.CLift, results.CDrag_induced, results.CM, results.CY, results.CL, results.CN)
        finally:
            write(*original)
            wing_planform(wing)

    report  = check_derivatives(planform_and_coefficients, original, steps=(1e-5, 1e-6, 1e-7), n_directions=2)
    reached = {key: float(np.max(np.abs(sensitivity))) for key, sensitivity in zip(names, report.input_sensitivity)}
    print(f"  {name} {wing.tag} planform, {sum(value.size for value in original)} numbers that define it ({', '.join(names)}): "
          f"values {report.value_error:.1e}, derivatives {report.jvp_error:.1e}, reverse {report.vjp_consistency_error:.1e}")
    assert report.passed, f"{name} {wing.tag}: wing planform JAX check failed, {report}"
    assert min(reached.values()) > 0, f"{name} {wing.tag}: numbers of the wing that do not reach the planform or the coefficients: {reached}"
    return

def check_control_surfaces(name, vehicle, settings):
    # the coefficients, differentiated with respect to the deflection and the chord fraction of every control surface
    # that moves panels of the lattice, with each surface deflected so that its chord fraction has an effect
    from RCAIDE.Library.Methods.Utilities.derivative_check import check_derivatives
    surfaces = [surface for wing in vehicle.wings for surface in wing.control_surfaces
                if not isinstance(surface, RCAIDE.Library.Components.Wings.Control_Surfaces.Spoiler)]
    layouts  = {wing.tag: wing_lattice_layout(wing, settings.number_of_chordwise_vortices, settings.number_of_spanwise_vortices,
                                              settings.spanwise_cosine_spacing) for wing in vehicle.wings}
    names    = ('deflections', 'chord fractions')
    stored   = (np.array([surface.deflection for surface in surfaces], dtype=float), np.array([surface.chord_fraction for surface in surfaces], dtype=float))
    original = (stored[0] + 5.0 * Units.deg, stored[1])

    def write(deflections, chord_fractions):
        for i, surface in enumerate(surfaces):
            surface.deflection     = deflections[i]
            surface.chord_fraction = chord_fractions[i]

    def coefficients(*arrays):
        try:
            write(*arrays)
            lattice = generate_vortex_distribution(flow_conditions(), settings, vehicle, layouts)
            results = VLM(flow_conditions(), settings, vehicle, VD=lattice)
            return results.CLift, results.CDrag_induced, results.CM, results.CY, results.CL, results.CN
        finally:
            write(*stored)

    report  = check_derivatives(coefficients, original, steps=(1e-5, 1e-6, 1e-7), n_directions=2)
    # every surface is checked on its own: its deflection and its chord fraction must each reach the coefficients
    unreached = [f"{surface.tag} {key}" for key, sensitivity in zip(names, report.input_sensitivity)
                 for surface, value in zip(surfaces, sensitivity) if value == 0]
    print(f"  {name} control surfaces, {len(surfaces)} surfaces ({', '.join(surface.tag for surface in surfaces)}): "
          f"values {report.value_error:.1e}, derivatives {report.jvp_error:.1e}, reverse {report.vjp_consistency_error:.1e}")
    assert report.passed, f"{name}: control surface JAX check failed, {report}"
    assert len(unreached) == 0, f"{name}: control surface numbers that do not reach the coefficients: {unreached}"
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def defining_numbers(wing):
    """The names of the numbers that define a wing, and functions that read them from the wing and write them to it."""
    segments = list(wing.segments.values())

    # a wing given without segments: wing_planform sets its span, root chord and two segments from these
    if wing.segments_from_planform:
        names = ('area', 'aspect ratio', 'taper', 'quarter-chord sweep', 'twists', 'dihedral')
        def read():
            return (np.array([wing.areas.reference], dtype=float), np.array([wing.aspect_ratio], dtype=float),
                    np.array([wing.taper], dtype=float), np.array([wing.sweeps.quarter_chord], dtype=float),
                    np.array([segment.twist for segment in segments], dtype=float), np.array([wing.dihedral], dtype=float))
        def write(area, aspect_ratio, taper, sweep, twists, dihedral=None):
            wing.areas.reference      = area[0]
            wing.aspect_ratio         = aspect_ratio[0]
            wing.taper                = taper[0]
            wing.sweeps.quarter_chord = sweep[0]
            for i, segment in enumerate(segments):
                segment.twist = twists[i]
            if dihedral is not None:
                wing.dihedral = dihedral[0]

        # the cant of a vertical surface is left out: the Trefftz-plane induced drag is not continuous in it at zero cant
        if wing.vertical:
            return names[:-1], lambda: read()[:-1], write
        return names, read, write

    # a wing given by segments; the last one is the tip, with no panel outboard of it, so no dihedral or sweep of its own
    names = ('span', 'root chord', 'chord fractions', 'twists', 'dihedrals', 'quarter-chord sweeps')
    def read():
        return (np.array([wing.spans.projected], dtype=float), np.array([wing.chords.root], dtype=float),
                np.array([segment.root_chord_percent for segment in segments], dtype=float),
                np.array([segment.twist for segment in segments], dtype=float),
                np.array([segment.dihedral_outboard for segment in segments[:-1]], dtype=float),
                np.array([segment.sweeps.quarter_chord for segment in segments[:-1]], dtype=float))
    def write(span, root_chord, chord_fractions, twists, dihedrals, sweeps):
        wing.spans.projected = span[0]
        wing.chords.root     = root_chord[0]
        for i, segment in enumerate(segments):
            segment.root_chord_percent = chord_fractions[i]
            segment.twist              = twists[i]
        for i, segment in enumerate(segments[:-1]):
            segment.dihedral_outboard    = dihedrals[i]
            segment.sweeps.quarter_chord = sweeps[i]
    return names, read, write


def free_entries(lattice):
    """The entries of the lateral coordinates that are inputs of a derivative check: all but those on a vertical
    surface, where a sideways move of one point alone changes which end of a vortex comes first."""
    vertical_panel = np.isclose(lattice.YAH, lattice.YBH)[0]
    node_sizes     = (lattice.n_sw[0] + 1) * (lattice.n_cw[0] + 1)
    vertical_node  = np.concatenate([np.full(len(nodes), np.ptp(nodes) < 1e-9)
                                     for nodes in np.split(np.asarray(lattice.Y)[0], np.cumsum(node_sizes)[:-1])])
    free = {key: np.flatnonzero(~vertical_panel)
            for key in ('YA1', 'YA2', 'YAC', 'YAH', 'YA_TE', 'YB1', 'YB2', 'YBC', 'YBH', 'YB_TE', 'YC', 'YCH')}
    free['Y'] = np.flatnonzero(~vertical_node)
    return free


def flow_conditions():
    conditions = control_surface_conditions(np.array([0.5]))
    conditions.aerodynamics.angles.alpha[:]   = 3.0 * Units.deg
    conditions.aerodynamics.angles.beta[:]    = 2.0 * Units.deg
    conditions.freestream.velocity[:]         = 170.0
    conditions.static_stability.pitch_rate[:] = 0.02
    conditions.static_stability.roll_rate[:]  = 0.03
    conditions.static_stability.yaw_rate[:]   = 0.01
    return conditions


def coarse_lattice(name):
    """A VnV vehicle, vortex lattice settings with 8 spanwise and 3 chordwise panels per surface, and its lattice."""
    vehicle          = __import__(name).vehicle_setup()
    analyses         = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle
    analyses.append(RCAIDE.Framework.Analyses.Geometry.Geometry())
    geometry_preprocess_routine(analyses)
    settings = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method().settings
    settings.number_of_spanwise_vortices  = 8
    settings.number_of_chordwise_vortices = 3
    return vehicle, settings, generate_vortex_distribution(control_surface_conditions(np.array([0.6])), settings, vehicle)


if __name__ == '__main__':
    main()
