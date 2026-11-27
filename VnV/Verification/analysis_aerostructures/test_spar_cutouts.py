# VnV/Verification/analysis_aerostructures/test_spar_cutouts.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                                                   import Data
from RCAIDE.Library.Methods.Geometry.Planform                                                import wing_planform
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.discretize_wing           import discretize_wing
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_wingbox_properties import compute_wingbox_properties, section_properties, wingbox_shape, kuhn_shear_stiffness_factor
import numpy as np
from scipy.integrate import quad

# front spar hole and the rib bay that holds it [m]; the web is 0.234 m deep
DIAMETER   = 0.14
BAY_LENGTH = 0.234

# ----------------------------------------------------------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    check_kuhn_factor_matches_table_1()
    check_toggle_off_is_unchanged()
    check_flexibility_is_mesh_independent()
    check_bay_uses_kuhn_thickness_in_torsion()
    check_severed_spar_raises()
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def build_wing():
    """Rectangular, unswept wing: 10 m semi-span, 2 m chord, 15% thick, spars at 10% and 60% chord, so the
    wingbox is 1.0 m wide everywhere and, with the NACA four-digit section, 0.234 m deep at the front spar."""
    wing                        = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                    = 'main_wing'
    wing.spans.projected        = 20.0
    wing.chords.root            = 2.0
    wing.chords.tip             = 2.0
    wing.taper                  = 1.0
    wing.areas.reference        = 40.0
    wing.aspect_ratio           = 10.0
    wing.thickness_to_chord     = 0.15
    wing.xz_plane_symmetric     = True
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
    wing.structural.front_spar.t_web = 0.004
    wing.structural.rear_spar.t_web  = 0.004

    # front spar hole: D/h = 0.6 and D/b = 0.6, inside Kuhn's tested range
    wing.structural.spar_cutouts = [Data(spar='front', percent_span_location=0.5, diameter=DIAMETER, bay_length=BAY_LENGTH)]
    return wing


def box_at_mid_span(wing):
    """Shape of the wing box section, the same along this wing: chordwise points and surface heights [m]."""
    x, z_upper, z_lower = wingbox_shape(wing, [5.0], [2.0], [0.1], [0.6], 10.0)
    return x[0], z_upper[0], z_lower[0]


def vertical_flexibility(wing, n_elem):
    """Integral of 1/Ixx along the span [1/m^3], summed over the elements of an n_elem mesh."""
    geom = discretize_wing(wing, n_elem)
    section = compute_wingbox_properties(wing, geom, spar_cutouts=True)
    return np.sum(geom.Le / section.Ixx)

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def check_kuhn_factor_matches_table_1():
    # Kuhn, NACA WR L-323 (1942), Table 1: specimen D, b, h [in] and the reported eta_0
    specimens = [(2.63, 4.38, 4.02, 0.288), (2.63, 5.75, 4.06, 0.396), (4.50, 7.00, 6.02, 0.207), (4.50, 7.75, 10.05, 0.381)]
    for D, b, h, eta_table in specimens:
        eta = kuhn_shear_stiffness_factor(D, b, h)
        assert abs(eta - eta_table) < 0.002, (D, b, h, eta, eta_table)
    return


def check_toggle_off_is_unchanged():
    wing        = build_wing()
    geom        = discretize_wing(wing, 100)
    with_holes  = compute_wingbox_properties(wing, geom, spar_cutouts=False)
    wing.structural.spar_cutouts = []
    no_holes    = compute_wingbox_properties(wing, geom, spar_cutouts=True)
    for key in ('A', 'Ixx', 'Izz', 'J', 'w_box', 'fiber_distance', 'enclosed_area'):
        assert np.array_equal(with_holes[key], no_holes[key])
    return


def check_flexibility_is_mesh_independent():
    wing  = build_wing()
    D, yc = DIAMETER, 5.0
    t     = wing.structural.front_spar.t_web

    # independent reference: integrate 1/Ixx(y) across the hole, where the web loses a centred cut of height c(y).
    # The section is symmetric, so the cut is centred on its centroid
    Ixx_full = section_properties(wing, *box_at_mid_span(wing)).Ixx
    def inverse_Ixx(y):
        c = D * np.sqrt(max(0.0, 1.0 - ((y - yc) / (D / 2))**2))
        return 1.0 / (Ixx_full - t * c**3 / 12)
    reference = 10.0 / Ixx_full + quad(inverse_Ixx, yc - D / 2, yc + D / 2)[0] - D / Ixx_full

    # coarse elements (0.25 m) are wider than the hole; fine elements (0.01 m) resolve it
    for n_elem in (40, 1000):
        flexibility = vertical_flexibility(wing, n_elem)
        assert abs(flexibility - reference) / abs(reference - 10.0 / Ixx_full) < 0.01, (n_elem, flexibility, reference)
    return


def check_bay_uses_kuhn_thickness_in_torsion():
    wing  = build_wing()
    geom  = discretize_wing(wing, 1000)
    section = compute_wingbox_properties(wing, geom, spar_cutouts=True)
    box     = box_at_mid_span(wing)
    h_front = box[1][0] - box[2][0]

    # an element inside the bay but outside the hole keeps its full web for bending and Kuhn's thickness for torsion
    e   = int(np.argmin(np.abs(geom.Y_elems - (5.0 + 0.4 * BAY_LENGTH))))
    eta = kuhn_shear_stiffness_factor(DIAMETER, BAY_LENGTH, h_front)
    J_expected = section_properties(wing, *box, eta_front=eta).J
    assert abs(section.J[e] - J_expected) / J_expected < 1e-10, (section.J[e], J_expected)
    assert abs(section.Ixx[e] - section_properties(wing, *box).Ixx) < 1e-12

    # outside the bay the section is untouched
    e_far = int(np.argmin(np.abs(geom.Y_elems - 8.0)))
    assert np.isclose(section.J[e_far], section_properties(wing, *box).J, rtol=1e-12)
    return


def check_severed_spar_raises():
    wing = build_wing()
    wing.structural.spar_cutouts = [Data(spar='rear', percent_span_location=0.5, diameter=0.31, bay_length=0.60)]
    geom = discretize_wing(wing, 100)
    try:
        compute_wingbox_properties(wing, geom, spar_cutouts=True)
    except ValueError:
        return
    raise AssertionError("a cutout deeper than the web must raise")


if __name__ == '__main__':
    main()
