# VnV/Verification/analysis_aerostructures/test_beam_closed_form.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                                          import Data, Units
from RCAIDE.Library.Methods.Geometry.Planform                                       import wing_planform
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.discretize_wing  import discretize_wing
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.FEA              import assemble_beam, nodal_force_vector, solve_beam, recover_stresses
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_wingbox_properties import section_properties, wingbox_shape
import numpy as np

# The closed forms below are the solutions of the Euler-Bernoulli beam equation E I w'''' = load, and of
# G J theta' = torque, for a uniform cantilever of length L clamped at s = 0.

SEMI_SPAN = 10.0
CHORD     = 2.0
SETTINGS  = Data(spar_cutouts=False)

# ----------------------------------------------------------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    check_tip_force()
    check_uniform_load()
    check_tip_torque()
    check_rectangular_section()
    check_tapered_section()
    check_box_lies_inside_the_wing()
    check_chordwise_tip_force()
    check_root_bending_stress()
    check_swept_beam()
    check_mesh_independence()
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def build_beam(sweep=0.0, elements=50):
    """Uniform rectangular wing, 10 m semi-span and 2 m chord, as a cantilever beam along its mid-spar line."""
    wing                         = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                     = 'main_wing'
    wing.spans.projected         = 2 * SEMI_SPAN
    wing.chords.root             = CHORD
    wing.chords.tip              = CHORD
    wing.taper                   = 1.0
    wing.areas.reference         = 2 * SEMI_SPAN * CHORD
    wing.aspect_ratio            = 2 * SEMI_SPAN / CHORD
    wing.thickness_to_chord      = 0.15
    wing.sweeps.quarter_chord    = sweep
    wing.xz_plane_symmetric      = True
    for tag, span_location in [('root', 0.0), ('tip', 1.0)]:
        segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                   = tag
        segment.percent_span_location = span_location
        segment.root_chord_percent    = 1.0
        segment.thickness_to_chord    = 0.15
        segment.sweeps.quarter_chord  = sweep
        segment.dihedral_outboard     = 0.0
        wing.append_segment(segment)
    wing_planform(wing)
    node_data = discretize_wing(wing, elements)
    beam      = assemble_beam(wing, node_data, SETTINGS)
    return wing, node_data, beam


def stiffnesses(wing, beam):
    E, G = wing.structural.material.youngs_modulus, wing.structural.material.shear_modulus
    return E * beam.Ixx[0], E * beam.Izz[0], G * beam.J[0]


def tip_load(beam, dof, value):
    """Nodal force vector with one load on one degree of freedom of the tip node."""
    F                        = np.zeros(beam.total_dof)
    F[beam.total_dof - 6 + dof] = value
    return F

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def check_tip_force():
    # w(s) = P s^2 (3L - s) / (6 E I), slope at the tip P L^2 / (2 E I)
    wing, nodes, beam = build_beam()
    EI, _, _          = stiffnesses(wing, beam)
    P, L, s           = 1.0e4, SEMI_SPAN, nodes.y_local
    u                 = solve_beam(beam, tip_load(beam, 2, P))
    assert np.allclose(u[2::6], P * s**2 * (3 * L - s) / (6 * EI), rtol=1e-9, atol=1e-12)
    assert abs(abs(u[-3]) - P * L**2 / (2 * EI)) < 1e-9 * P * L**2 / (2 * EI)     # rotation about x at the tip
    return


def check_uniform_load():
    # w(s) = w_0 s^2 (6 L^2 - 4 L s + s^2) / (24 E I)
    wing, nodes, beam = build_beam()
    EI, _, _          = stiffnesses(wing, beam)
    w_0, L, s         = 2.0e3, SEMI_SPAN, nodes.y_local
    zeros             = np.zeros_like(nodes.Le)
    u                 = solve_beam(beam, nodal_force_vector(beam, nodes, zeros, zeros, w_0 + zeros, zeros))
    assert np.allclose(u[2::6], w_0 * s**2 * (6 * L**2 - 4 * L * s + s**2) / (24 * EI), rtol=1e-9, atol=1e-12)
    return


def check_rectangular_section():
    # a box of width w and height h with flat skins. Bending: skins of thickness t_s (skin plus smeared stringers)
    # at h/2 and webs of thickness t_w. Torsion: Bredt-Batho, J = 4 (w h)^2 / (sum of side length over wall
    # thickness), where the walls are the skins and the webs alone; the smeared stringers carry no shear flow
    wing      = RCAIDE.Library.Components.Wings.Main_Wing()
    structure = wing.structural
    assert structure.stringer_smeared_thickness > 0
    t_s, t_w  = structure.top_skin_thickness + structure.stringer_smeared_thickness, structure.front_spar.t_web
    w, h      = 1.0, 0.3
    section   = section_properties(wing, [0, w], [h / 2, h / 2], [-h / 2, -h / 2])
    assert np.isclose(section.A,   2 * w * t_s + 2 * h * t_w, rtol=1e-12)
    assert np.isclose(section.Ixx, 2 * (w * t_s**3 / 12 + w * t_s * (h / 2)**2) + 2 * t_w * h**3 / 12, rtol=1e-12)
    assert np.isclose(section.Izz, 2 * t_s * w**3 / 12 + 2 * h * t_w**3 / 12, rtol=1e-12)
    assert np.isclose(section.J,   4 * (w * h)**2 / (2 * w / structure.top_skin_thickness + 2 * h / t_w), rtol=1e-12)
    assert np.isclose(section.fiber_distance, h / 2, rtol=1e-12) and np.isclose(section.enclosed_area, w * h, rtol=1e-12)
    return


def check_tapered_section():
    # a box that is h_1 deep at the front spar and h_2 at the rear, symmetric about the chord line. Each skin is a
    # straight strip of length L from height h_1/2 to h_2/2, whose second moment about the chord line is
    # t L (z_1^2 + z_1 z_2 + z_2^2) / 3, plus its thickness about its own midline
    wing      = RCAIDE.Library.Components.Wings.Main_Wing()
    structure = wing.structural
    t_s, t_w  = structure.top_skin_thickness + structure.stringer_smeared_thickness, structure.front_spar.t_web
    w, h_1, h_2 = 1.0, 0.4, 0.2
    L         = np.hypot(w, (h_1 - h_2) / 2)
    I_skin    = t_s * L * ((h_1 / 2)**2 + (h_1 / 2) * (h_2 / 2) + (h_2 / 2)**2) / 3 + t_s**3 * w**2 / L / 12
    section   = section_properties(wing, [0, w], [h_1 / 2, h_2 / 2], [-h_1 / 2, -h_2 / 2])
    assert np.isclose(section.A,   2 * t_s * L + t_w * (h_1 + h_2), rtol=1e-12)
    assert np.isclose(section.Ixx, 2 * I_skin + t_w * h_1**3 / 12 + t_w * h_2**3 / 12, rtol=1e-12)
    assert np.isclose(section.J,   4 * (w * (h_1 + h_2) / 2)**2 / (2 * L / structure.top_skin_thickness + (h_1 + h_2) / t_w), rtol=1e-12)
    assert np.isclose(section.fiber_distance, h_1 / 2, rtol=1e-12)

    # moving the box off the chord line changes nothing: the inertia is taken about the centroid
    lifted    = section_properties(wing, [0, w], [h_1 / 2 + 0.1, h_2 / 2 + 0.1], [-h_1 / 2 + 0.1, -h_2 / 2 + 0.1])
    assert np.isclose(lifted.Ixx, section.Ixx, rtol=1e-12) and np.isclose(lifted.fiber_distance, section.fiber_distance, rtol=1e-12)
    return


def check_box_lies_inside_the_wing():
    # the wing has no airfoil attached, so its section is the NACA four-digit thickness distribution at 15%
    # (Abbott and von Doenhoff, Theory of Wing Sections, Eq. 6.2). The skins of the box must lie on it, the webs
    # must be as deep as the section at the spars, and nothing may reach beyond half the largest thickness
    wing, nodes, beam = build_beam()
    x, z_upper, z_lower = wingbox_shape(wing, nodes.Y_elems, nodes.chord_elems, nodes.spar_f_elems, nodes.spar_r_elems, nodes.total_span)
    half_thickness    = naca_half_thickness(x / CHORD, 0.15) * CHORD
    assert np.allclose(z_upper,  half_thickness, atol=1e-4 * CHORD)
    assert np.allclose(z_lower, -half_thickness, atol=1e-4 * CHORD)
    assert np.all(beam.fiber_distance <= 0.15 * CHORD / 2 * (1 + 1e-3))
    assert np.all(beam.enclosed_area < beam.w_box * 0.15 * CHORD)
    return


def naca_half_thickness(x, thickness_to_chord):
    return 5 * thickness_to_chord * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x**2 + 0.2843 * x**3 - 0.1015 * x**4)


def check_tip_torque():
    # theta(s) = T s / (G J)
    wing, nodes, beam = build_beam()
    _, _, GJ          = stiffnesses(wing, beam)
    T, s              = 5.0e3, nodes.y_local
    u                 = solve_beam(beam, tip_load(beam, 4, T))
    assert np.allclose(u[4::6], T * s / GJ, rtol=1e-9, atol=1e-14)
    assert np.max(np.abs(u[2::6])) < 1e-12        # torsion does not bend an unswept beam
    return


def check_chordwise_tip_force():
    # the same cubic in the chordwise plane, on E I_zz
    wing, nodes, beam = build_beam()
    _, EIz, _         = stiffnesses(wing, beam)
    P, L, s           = 1.0e4, SEMI_SPAN, nodes.y_local
    u                 = solve_beam(beam, tip_load(beam, 0, P))
    assert np.allclose(u[0::6], P * s**2 * (3 * L - s) / (6 * EIz), rtol=1e-9, atol=1e-12)
    return


def check_root_bending_stress():
    # a tip force P gives a root moment P L and an outer-fibre stress P L c / I, with c half the largest thickness
    # of the section between the spars (at 30% of chord for the NACA four-digit thickness distribution)
    wing, nodes, beam = build_beam()
    P, L              = 1.0e4, SEMI_SPAN
    zeros             = np.zeros_like(nodes.Le)
    u                 = solve_beam(beam, tip_load(beam, 2, P))
    normal_stress, shear_stress, margin = recover_stresses(wing, nodes, beam, u, zeros, zeros, zeros, zeros)
    c                 = naca_half_thickness(0.30, 0.15) * CHORD
    expected          = P * L * c / beam.Ixx[0]
    assert abs(normal_stress[0] - expected) < 5e-4 * expected, (normal_stress[0], expected)     # the section is read from 201 airfoil points
    assert np.all(np.diff(normal_stress) < 0)     # the moment falls linearly to the tip
    expected_margin   = wing.structural.material.yield_tensile_strength / expected - 1
    assert abs(margin[0] - expected_margin) < 1e-3 * expected_margin, (margin[0], expected_margin)
    return


def check_swept_beam():
    # a beam swept by Lambda is a cantilever of length L / cos(Lambda): a tip force P bends it by
    # P L_b^3 / (3 E I), and its tip slope P L_b^2 / (2 E I) is a rotation about the in-plane normal to the beam,
    # with components cos(Lambda) about x and sin(Lambda) about y. The y component is the nose-down wash-out.
    sweep             = 30.0 * Units.degrees
    wing, nodes, beam = build_beam(sweep=sweep)
    EI, _, _          = stiffnesses(wing, beam)
    P                 = 1.0e4
    L_b               = SEMI_SPAN / np.cos(sweep)
    assert abs(nodes.y_local[-1] - L_b) < 1e-9 * L_b
    u                 = solve_beam(beam, tip_load(beam, 2, P))
    slope             = P * L_b**2 / (2 * EI)
    assert abs(u[-4] - P * L_b**3 / (3 * EI)) < 1e-9 * P * L_b**3 / (3 * EI)
    assert abs(abs(u[-3]) - slope * np.cos(sweep)) < 1e-9 * slope
    assert abs(abs(u[-2]) - slope * np.sin(sweep)) < 1e-9 * slope
    assert u[-2] < 0.0                            # an upward load twists a swept-back wing nose down
    return


def check_mesh_independence():
    # cubic elements carry these loads exactly, so the tip deflection does not depend on the mesh. Round-off grows
    # with the number of elements through the conditioning of the stiffness matrix (2e-6 at 400 elements).
    tips = []
    for elements in (5, 50, 400):
        wing, nodes, beam = build_beam(elements=elements)
        tips.append(solve_beam(beam, tip_load(beam, 2, 1.0e4))[-4])
    assert abs(tips[0] - tips[1]) < 1e-8 * abs(tips[1]), tips
    assert abs(tips[2] - tips[1]) < 5e-6 * abs(tips[1]), tips
    return


if __name__ == '__main__':
    main()
    print('test_beam_closed_form passed')
