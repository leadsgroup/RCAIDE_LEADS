# VnV/Verification/plots/test_generate_3d_wing_points.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                        import Units
from RCAIDE.Library.Methods.Geometry.Planform                     import wing_planform
from RCAIDE.Library.Plots.Geometry.generate_3d_wing_points        import generate_3d_wing_points
import numpy as np

# Deflected control surfaces in the points of a drawn wing, against hand calculations on a rectangular wing of
# chord 2 m and semispan 10 m with one control surface of 0.3 chord between 20% and 60% of the semispan. The hinge
# is at x = 1.4 m on the chord line, and the surface is 0.6 m long.

CHORD          = 2.0
SEMI_SPAN      = 10.0
HINGE          = 1.4
SURFACE_LENGTH = 0.6
POINTS         = 101

# ----------------------------------------------------------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    check_undeflected_surface_changes_nothing()
    check_flap_rotates_about_its_hinge()
    check_aileron_reverses_on_the_mirrored_side()
    check_flap_is_the_same_on_both_sides()
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def rectangular_wing(surface_class=None, deflection=0.0):
    wing                     = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.spans.projected     = 2 * SEMI_SPAN
    wing.chords.root         = CHORD
    wing.chords.tip          = CHORD
    wing.taper               = 1.0
    wing.areas.reference     = 2 * SEMI_SPAN * CHORD
    wing.aspect_ratio        = 2 * SEMI_SPAN / CHORD
    wing.thickness_to_chord  = 0.12
    wing.xz_plane_symmetric  = True
    for tag, span_location in (('root', 0.0), ('tip', 1.0)):
        segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                   = tag
        segment.percent_span_location = span_location
        segment.root_chord_percent    = 1.0
        segment.thickness_to_chord    = 0.12
        segment.sweeps.leading_edge   = 0.0
        segment.dihedral_outboard     = 0.0
        segment.twist                 = 0.0
        wing.append_segment(segment)
    if surface_class is not None:
        surface                     = surface_class()
        surface.tag                 = 'surface'
        surface.span_fraction_start = 0.2
        surface.span_fraction_end   = 0.6
        surface.chord_fraction      = SURFACE_LENGTH / CHORD
        surface.deflection          = deflection
        wing.append_control_surface(surface)
    wing_planform(wing)
    return wing


def rotated_trailing_edge(angle):
    """Where the lower trailing-edge point of the clean section goes when the surface turns by angle, trailing
    edge down: the point is (SURFACE_LENGTH, z) from the hinge, z being half the trailing-edge thickness."""
    z = generate_3d_wing_points(rectangular_wing(), POINTS).PTS[0, 0, 2]
    return np.array([HINGE + SURFACE_LENGTH * np.cos(angle) + z * np.sin(angle), -SURFACE_LENGTH * np.sin(angle) + z * np.cos(angle)])

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_undeflected_surface_changes_nothing():
    # a surface at zero deflection leaves the wing exactly as it is drawn without one
    clean = generate_3d_wing_points(rectangular_wing(), POINTS).PTS
    flap  = generate_3d_wing_points(rectangular_wing(RCAIDE.Library.Components.Wings.Control_Surfaces.Flap, 0.0), POINTS).PTS
    assert clean.shape == flap.shape and np.array_equal(clean, flap)
    return


def check_flap_rotates_about_its_hinge():
    angle  = 30 * Units.degrees
    clean  = generate_3d_wing_points(rectangular_wing(), POINTS).PTS
    points = generate_3d_wing_points(rectangular_wing(RCAIDE.Library.Components.Wings.Control_Surfaces.Flap, angle), POINTS).PTS

    # sections: root, clean and deflected at 2 m, deflected and clean at 6 m, tip and tip cap
    assert np.allclose(points[:, 0, 1], [0.0, 2.0, 2.0, 6.0, 6.0, 10.0, 10.0])
    expected = rotated_trailing_edge(angle)
    for row in (2, 3):
        assert np.allclose(points[row, 0, [0, 2]], expected, atol=1e-9)
    for row in (0, 1, 4, 5):
        assert np.allclose(points[row, 0, [0, 2]], clean[0, 0, [0, 2]], atol=1e-12)

    # ahead of the hinge the section is untouched, and aft of it every point keeps its distance from the hinge
    ahead = clean[0][:, 0] < HINGE
    assert np.allclose(points[2][ahead][:, [0, 2]], clean[0][ahead][:, [0, 2]], atol=1e-12)
    hinge = np.array([HINGE, 0.0])
    assert np.allclose(np.linalg.norm(points[2][~ahead][:, [0, 2]] - hinge, axis=1), np.linalg.norm(clean[0][~ahead][:, [0, 2]] - hinge, axis=1), atol=1e-9)
    return


def check_aileron_reverses_on_the_mirrored_side():
    angle = 20 * Units.degrees
    wing  = rectangular_wing(RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron, angle)
    right = generate_3d_wing_points(wing, POINTS).PTS
    left  = generate_3d_wing_points(wing, POINTS, mirrored=True).PTS
    assert np.allclose(right[2, 0, [0, 2]], rotated_trailing_edge(angle),  atol=1e-9)       # trailing edge down
    assert np.allclose(left[2, 0, [0, 2]],  rotated_trailing_edge(-angle), atol=1e-9)       # trailing edge up
    return


def check_flap_is_the_same_on_both_sides():
    wing = rectangular_wing(RCAIDE.Library.Components.Wings.Control_Surfaces.Flap, 30 * Units.degrees)
    assert np.array_equal(generate_3d_wing_points(wing, POINTS).PTS, generate_3d_wing_points(wing, POINTS, mirrored=True).PTS)
    return


if __name__ == '__main__':
    main()
    print('test_generate_3d_wing_points passed')
