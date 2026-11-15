# VnV/Verification/geometry/test_wing_planform.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Geometry.Planform import wing_planform

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # wing_planform computes a wing's planform values. A wing can be given by its reference area, aspect ratio, taper
    # and sweep, or by its span, root chord and segments. Both must give the same wing, a second pass must leave it
    # unchanged, a wing without a mirror image (a vertical tail) must use its one side, and a changed definition must
    # be picked up at the next pass.
    check_trapezoid_against_closed_form(horizontal_tail(), mirrored=True)
    check_trapezoid_against_closed_form(vertical_tail(), mirrored=False)
    check_second_pass_unchanged(horizontal_tail())
    check_second_pass_unchanged(vertical_tail())
    check_same_wing_from_segments(horizontal_tail())
    check_same_wing_from_segments(vertical_tail())
    check_changed_definition_is_used(horizontal_tail())
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_trapezoid_against_closed_form(wing, mirrored):
    area, aspect_ratio, taper = wing.areas.reference, wing.aspect_ratio, wing.taper
    sweep, thickness          = wing.sweeps.quarter_chord, wing.thickness_to_chord
    wing_planform(wing)

    # a trapezoid: the span is the full span of a mirrored wing and the height of a one-sided one
    span       = np.sqrt(aspect_ratio * area)
    panel_span = span / 2 if mirrored else span
    root_chord = 2 * area / (span * (1 + taper))
    tip_chord  = taper * root_chord

    # the leading edge is a quarter chord ahead of the quarter-chord line at the root and at the tip
    leading_edge_sweep = np.arctan(np.tan(sweep) + (root_chord - tip_chord) / (4 * panel_span))
    expected = {'span':               (wing.spans.projected,         span),
                'root chord':         (wing.chords.root,             root_chord),
                'tip chord':          (wing.chords.tip,              tip_chord),
                'reference area':     (wing.areas.reference,         area),
                'aspect ratio':       (wing.aspect_ratio,            aspect_ratio),
                'thickness ratio':    (wing.thickness_to_chord,      thickness),
                'frontal area':       (wing.areas.front_projected,   area * thickness),
                'leading-edge sweep': (wing.sweeps.leading_edge,     leading_edge_sweep),
                'total length':       (wing.total_length,            np.tan(leading_edge_sweep) * panel_span + tip_chord)}
    for name, (computed, value) in expected.items():
        assert abs(computed - value) <= 1e-12 * max(1.0, abs(value)), f"{wing.tag} {name}: {computed} against {value}"
    print(f"  {wing.tag}: {len(expected)} planform values match the trapezoid formulas")
    return


def check_second_pass_unchanged(wing):
    wing_planform(wing)
    first = planform_values(wing)
    wing_planform(wing)
    second = planform_values(wing)
    for name in first:
        assert np.allclose(first[name], second[name], rtol=1e-13, atol=1e-13), f"{wing.tag} {name}: {first[name]} then {second[name]}"
    print(f"  {wing.tag}: a second pass leaves {len(first)} planform values unchanged")
    return


def check_same_wing_from_segments(wing):
    # the same trapezoid, given by its span, root chord and two segments
    wing_planform(wing)
    segmented                         = type(wing)()
    segmented.tag                     = wing.tag + '_from_segments'
    segmented.vertical                = wing.vertical
    segmented.xz_plane_symmetric      = wing.xz_plane_symmetric
    segmented.spans.projected         = wing.spans.projected
    segmented.chords.root             = wing.chords.root
    segmented.thickness_to_chord      = wing.thickness_to_chord

    segment                           = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                       = 'root'
    segment.percent_span_location     = 0.0
    segment.root_chord_percent        = 1.0
    segment.sweeps.quarter_chord      = wing.sweeps.quarter_chord
    segment.dihedral_outboard         = wing.dihedral
    segment.thickness_to_chord        = wing.thickness_to_chord
    segmented.append_segment(segment)

    segment                           = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                       = 'tip'
    segment.percent_span_location     = 1.0
    segment.root_chord_percent        = wing.taper
    segment.thickness_to_chord        = wing.thickness_to_chord
    segmented.append_segment(segment)
    wing_planform(segmented)

    given, built = planform_values(wing), planform_values(segmented)
    for name in given:
        assert np.allclose(given[name], built[name], rtol=1e-12, atol=1e-12), f"{wing.tag} {name}: {given[name]} against {built[name]} from segments"
    print(f"  {wing.tag}: the same wing given by segments has the same {len(given)} planform values")
    return


def check_changed_definition_is_used(wing):
    wing_planform(wing)
    wing.aspect_ratio = 1.2 * wing.aspect_ratio
    wing_planform(wing)
    span = np.sqrt(wing.aspect_ratio * wing.areas.reference)
    assert abs(wing.spans.projected - span) <= 1e-12 * span, f"{wing.tag}: span {wing.spans.projected} after the aspect ratio changed, expected {span}"
    assert abs(wing.segments.tip.root_chord_percent - wing.taper) <= 1e-15
    print(f"  {wing.tag}: a changed aspect ratio gives the new span at the next pass")
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Wings
# ----------------------------------------------------------------------------------------------------------------------
def planform_values(wing):
    return {'span':                wing.spans.projected,
            'total span':          wing.spans.total,
            'root chord':          wing.chords.root,
            'tip chord':           wing.chords.tip,
            'mean chord':          wing.chords.mean_aerodynamic,
            'reference area':      wing.areas.reference,
            'frontal area':        wing.areas.front_projected,
            'wetted area':         wing.areas.wetted,
            'aspect ratio':        wing.aspect_ratio,
            'taper':               wing.taper,
            'thickness ratio':     wing.thickness_to_chord,
            'quarter-chord sweep': wing.sweeps.quarter_chord,
            'leading-edge sweep':  wing.sweeps.leading_edge,
            'total length':        wing.total_length,
            'aerodynamic center':  np.array(wing.aerodynamic_center, dtype=float)}


def horizontal_tail():
    wing                       = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag                   = 'horizontal_tail'
    wing.areas.reference       = 26.0
    wing.aspect_ratio          = 5.5
    wing.taper                 = 0.2
    wing.sweeps.quarter_chord  = 34.5 * Units.deg
    wing.thickness_to_chord    = 0.11
    wing.dihedral              = 8.4 * Units.deg
    return wing


def vertical_tail():
    wing                       = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag                   = 'vertical_tail'
    wing.areas.reference       = 16.0
    wing.aspect_ratio          = 1.7
    wing.taper                 = 0.31
    wing.sweeps.quarter_chord  = 35. * Units.deg
    wing.thickness_to_chord    = 0.10
    wing.vertical              = True
    wing.xz_plane_symmetric    = False
    return wing


if __name__ == '__main__':
    main()
