# VnV/Verification/analysis_aerodynamics/test_flap_profile_drag.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                            import Units
from RCAIDE.Library.Methods.Aerodynamics.Common.Drag  import compute_flap_profile_drag
from RCAIDE.Library.Methods.Aerodynamics.Common.Drag.compute_flap_profile_drag import FLAP_INTERFERENCE_FACTOR
import numpy as np
import pytest

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def build_wing(flap_type, chord_fraction, deflection_deg, sweep_deg=0.0, flapped_area=1.0, thickness_to_chord=0.12):
    wing                       = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.thickness_to_chord    = thickness_to_chord
    wing.sweeps.quarter_chord  = sweep_deg * Units.deg
    wing.areas.reference       = 1.0
    wing.areas.affected        = flapped_area
    flap                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.configuration_type    = flap_type
    flap.chord_fraction        = chord_fraction
    flap.deflection            = deflection_deg * Units.deg
    return wing, flap


def two_dimensional_increment(flap_type, chord_fraction, deflection_deg, thickness_to_chord=0.12):
    """Chart increment: unswept, fully flapped wing, interference factor removed."""
    wing, flap = build_wing(flap_type, chord_fraction, deflection_deg, thickness_to_chord=thickness_to_chord)
    return compute_flap_profile_drag(wing, flap, 1.0) / (1 + FLAP_INTERFERENCE_FACTOR[flap_type])

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
# Readings taken by eye from Roskam, Airplane Design Part VI (DARcorporation, 2004 printing), independently of the
# pixel digitization behind the tables: (flap type, c_f/c, deflection [deg], delta c_dp, figure)
CHART_READINGS = [
    ('double_slotted', 0.20, 30.0, 0.038, 'Fig. 4.47'),
    ('double_slotted', 0.30, 40.0, 0.090, 'Fig. 4.47'),
    ('double_slotted', 0.40, 30.0, 0.107, 'Fig. 4.47'),
    ('single_slotted', 0.30, 40.0, 0.066, 'Fig. 4.46'),
    ('single_slotted', 0.20, 50.0, 0.059, 'Fig. 4.46'),
    ('single_slotted', 0.10, 50.0, 0.025, 'Fig. 4.46'),
    ('fowler',         0.30, 40.0, 0.064, 'Fig. 4.48'),
    ('fowler',         0.40, 30.0, 0.064, 'Fig. 4.48'),
    ('fowler',         0.10, 40.0, 0.017, 'Fig. 4.48'),
    ('plain',          0.10, 60.0, 0.062, 'Fig. 4.44'),
    ('plain',          0.20, 15.0, 0.017, 'Fig. 4.44'),
]

# Fig. 4.45 split flaps, plotted points read by eye: (c_f/c, deflection [deg], t/c, delta c_dp)
SPLIT_READINGS = [(0.30, 60.0, 0.10, 0.280), (0.30, 60.0, 0.30, 0.205), (0.20, 15.0, 0.10, 0.027), (0.30, 15.0, 0.30, 0.019)]

@pytest.mark.parametrize("flap_type, chord_fraction, deflection, expected, figure", CHART_READINGS)
def check_two_dimensional_increment_matches_chart(flap_type, chord_fraction, deflection, expected, figure):
    assert abs(two_dimensional_increment(flap_type, chord_fraction, deflection) - expected) < 0.004, figure


@pytest.mark.parametrize("chord_fraction, deflection, thickness_to_chord, expected", SPLIT_READINGS)
def check_split_flap_increment_matches_chart(chord_fraction, deflection, thickness_to_chord, expected):
    assert abs(two_dimensional_increment('split', chord_fraction, deflection, thickness_to_chord) - expected) < 0.004


def check_split_flap_thinner_airfoil_has_more_drag():
    assert two_dimensional_increment('split', 0.25, 45.0, 0.10) > two_dimensional_increment('split', 0.25, 45.0, 0.20) > two_dimensional_increment('split', 0.25, 45.0, 0.30)


def check_increment_monotone_and_zero_at_zero_deflection():
    for flap_type in ('plain', 'split', 'single_slotted', 'double_slotted', 'fowler'):
        assert two_dimensional_increment(flap_type, 0.25, 0.0) == 0.0
        by_deflection = [two_dimensional_increment(flap_type, 0.25, d) for d in np.arange(0, 46, 5)]
        by_chord      = [two_dimensional_increment(flap_type, c, 35.0) for c in np.arange(0.0, 0.41, 0.05)]
        assert np.all(np.diff(by_deflection) > 0) and np.all(np.diff(by_chord) >= 0)


def check_three_dimensional_scaling():
    # Roskam Part VI Eqs. 4.71 and 4.75: (1 + K_int) * delta c_dp * cos(sweep) * S_wf / S
    wing, flap  = build_wing('double_slotted', 0.30, 40.0, sweep_deg=25.0, flapped_area=0.5)
    expected    = (1 + 0.40) * two_dimensional_increment('double_slotted', 0.30, 40.0) * np.cos(25.0 * Units.deg) * 0.5 / 2.0
    assert np.isclose(compute_flap_profile_drag(wing, flap, 2.0), expected, rtol=1e-12)


def check_uncharted_flap_types_fall_back_with_a_warning():
    with pytest.warns(UserWarning):
        wing, flap = build_wing('triple_slotted', 0.25, 30.0)
        triple = compute_flap_profile_drag(wing, flap, 1.0)
    assert np.isclose(triple, (1 + 0.40) * two_dimensional_increment('double_slotted', 0.25, 30.0))
    with pytest.warns(UserWarning):
        wing, flap = build_wing('slotted_and_unknown', 0.25, 30.0)
        unknown = compute_flap_profile_drag(wing, flap, 1.0)
    assert np.isclose(unknown, (1 + 0.40) * two_dimensional_increment('single_slotted', 0.25, 30.0))


def main():
    for reading in CHART_READINGS:
        check_two_dimensional_increment_matches_chart(*reading)
    for reading in SPLIT_READINGS:
        check_split_flap_increment_matches_chart(*reading)
    check_split_flap_thinner_airfoil_has_more_drag()
    check_increment_monotone_and_zero_at_zero_deflection()
    check_three_dimensional_scaling()
    check_uncharted_flap_types_fall_back_with_a_warning()
    return


if __name__ == '__main__':
    main()
