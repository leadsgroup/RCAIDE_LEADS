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
    # a tip force P gives a root moment P L and an outer-fibre stress P L (h / 2) / I
    wing, nodes, beam = build_beam()
    P, L              = 1.0e4, SEMI_SPAN
    zeros             = np.zeros_like(nodes.Le)
    u                 = solve_beam(beam, tip_load(beam, 2, P))
    normal_stress, shear_stress, margin = recover_stresses(wing, nodes, beam, u, zeros, zeros, zeros, zeros)
    h                 = CHORD * 0.15
    expected          = P * L * (h / 2) / beam.Ixx[0]
    assert abs(normal_stress[0] - expected) < 1e-9 * expected, (normal_stress[0], expected)
    assert np.all(np.diff(normal_stress) < 0)     # the moment falls linearly to the tip
    expected_margin   = wing.structural.material.yield_tensile_strength / expected - 1
    assert abs(margin[0] - expected_margin) < 1e-9 * expected_margin, (margin[0], expected_margin)
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
    # with the number of elements through the conditioning of the stiffness matrix (7e-8 at 400 elements).
    tips = []
    for elements in (5, 50, 400):
        wing, nodes, beam = build_beam(elements=elements)
        tips.append(solve_beam(beam, tip_load(beam, 2, 1.0e4))[-4])
    assert abs(tips[0] - tips[1]) < 1e-8 * abs(tips[1]), tips
    assert abs(tips[2] - tips[1]) < 1e-6 * abs(tips[1]), tips
    return


if __name__ == '__main__':
    main()
    print('test_beam_closed_form passed')
