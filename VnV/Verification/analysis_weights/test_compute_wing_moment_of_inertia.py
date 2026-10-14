# VnV/Verification/analysis_weights/test_compute_wing_moment_of_inertia.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

import RCAIDE
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia import compute_wing_moment_of_inertia

# ----------------------------------------------------------------------------------------------------------------------
#  Rectangular wing: 20 m span, 2 m chord, 12 percent thick, 1000 kg
# ----------------------------------------------------------------------------------------------------------------------
def rectangular_wing():
    wing                      = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.mass_properties.mass = 1000.0
    wing.spans.projected      = 20.0
    wing.chords.root          = 2.0
    wing.chords.tip           = 2.0
    wing.thickness_to_chord   = 0.12
    wing.sweeps.quarter_chord = 0.0
    return wing


def add_segment(wing, tag, percent_span_location):
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = tag
    segment.percent_span_location = percent_span_location
    segment.root_chord_percent    = 1.0
    segment.thickness_to_chord    = 0.12
    segment.sweeps.quarter_chord  = 0.0
    wing.append_segment(segment)


# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def test_segmented_wing_matches_unsegmented_wing():
    I_plain, _ = compute_wing_moment_of_inertia(rectangular_wing())

    segmented = rectangular_wing()
    add_segment(segmented, 'root', 0.0)
    add_segment(segmented, 'root_duplicate', 0.0)      # zero-span section is skipped
    add_segment(segmented, 'tip', 1.0)
    I_segmented, _ = compute_wing_moment_of_inertia(segmented)

    np.testing.assert_allclose(I_segmented, I_plain, rtol=1e-12, atol=1e-9)


def test_unsegmented_wing_rolls_harder_than_it_pitches():
    I, mass = compute_wing_moment_of_inertia(rectangular_wing())
    assert mass == 1000.0
    assert I[0, 0] > I[1, 1] > 0                      # spanwise mass spread dominates roll inertia
