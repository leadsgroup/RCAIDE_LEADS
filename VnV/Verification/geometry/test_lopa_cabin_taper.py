# VnV/Verification/geometry/test_lopa_cabin_taper.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Geometry.LOPA.compute_layout_of_passenger_accommodations import compute_layout_of_passenger_accommodations

# ----------------------------------------------------------------------------------------------------------------------
#  REFERENCE (hand calculation)
# ----------------------------------------------------------------------------------------------------------------------
# One economy class of 10 rows, 6 abreast, 32 in pitch, 18 in seats, 2 in armrests, 15 in aisle (class defaults).
#   cabin length  = 10 x 32 = 320 in; row centers at 16, 48, ..., 304 in; each seat spans center -/+ 9 in
#   class width   = 8 armrests x 2 + 6 x 18 + 15 = 139 in
#   seat outboard edges, measured from the inboard edge of the side cabin: 137, 117, 97, 60, 40, 20 in
# Tail fineness 1: wedge length 139 in, so rows with center > 320 - 139 = 181 in are checked.
#   border at a row = (320 - seat front) x 139 / 139 = 329 - center; a seat is removed if its outboard edge is beyond it
#   row 304: border  25 -> 5 removed     row 272: border  57 -> 4 removed
#   row 240: border  89 -> 3 removed     row 208: border 121 -> 1 removed
#   13 seats per side, 26 for both sides: the side cabin keeps 120 - 26 = 94 seats, the main cabin all 60
SEATS_UNTAPERED  = 180
SEATS_TAIL_1     = 154
REMOVED_PER_ROW  = {304: 5, 272: 4, 240: 3, 208: 1}   # [in] row center -> seats removed per side


# ----------------------------------------------------------------------------------------------------------------------
#  MAIN
# ----------------------------------------------------------------------------------------------------------------------
def main():
    check_untapered_side_cabin_keeps_every_seat()
    check_tail_taper_matches_hand_calculation()
    check_tail_taper_removes_only_aft_seats()
    return


def check_untapered_side_cabin_keeps_every_seat():
    """With no taper the side cabin keeps all of its seats."""
    lopa = layout(tail_fineness=0.0)
    assert int(lopa[:, 10].sum()) == SEATS_UNTAPERED
    return


def check_tail_taper_matches_hand_calculation():
    """Tail fineness 1 removes the hand-calculated number of seats from each aft row."""
    lopa  = layout(tail_fineness=1.0)
    assert int(lopa[:, 10].sum()) == SEATS_TAIL_1

    # removed seats per row, one side of the side cabin
    full    = side_seats(layout(tail_fineness=0.0))
    tapered = side_seats(lopa)
    for center, n_removed in REMOVED_PER_ROW.items():
        x = center * Units.inches
        n_full    = np.sum(np.isclose(full[:, 2], x))
        n_tapered = np.sum(np.isclose(tapered[:, 2], x))
        assert (n_full - n_tapered) // 2 == n_removed
    return


def check_tail_taper_removes_only_aft_seats():
    """The tail taper leaves the rows ahead of the wedge, and the main cabin, untouched."""
    full    = layout(tail_fineness=0.0)
    tapered = layout(tail_fineness=1.0)
    wedge_start = (320 - 139) * Units.inches

    # main cabin and the side-cabin rows ahead of the wedge are identical
    ahead = lambda lopa: lopa[(lopa[:, 2] < wedge_start) | ~is_side(lopa)]
    assert ahead(full).shape == ahead(tapered).shape
    assert np.allclose(ahead(full), ahead(tapered))
    return


# ----------------------------------------------------------------------------------------------------------------------
#  HELPERS
# ----------------------------------------------------------------------------------------------------------------------
def layout(tail_fineness):
    """LOPA of a main cabin and a side cabin, each one economy class of 10 rows, 6 abreast."""
    fuselage = RCAIDE.Library.Components.Fuselages.Fuselage()
    for cabin in [RCAIDE.Library.Components.Fuselages.Cabins.Cabin(), RCAIDE.Library.Components.Fuselages.Cabins.Side_Cabin()]:
        economy = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy()
        economy.number_of_seats_abrest = 6
        economy.number_of_rows         = 10
        cabin.append_cabin_class(economy)
        fuselage.append_cabin(cabin)
    fuselage.cabins.side_cabin.tail.fineness_ratio = tail_fineness
    compute_layout_of_passenger_accommodations(fuselage)
    return np.asarray(fuselage.layout_of_passenger_accommodations.object_coordinates)


def is_side(lopa):
    """Objects of the side cabin, which lies outboard of the 139 in main cabin."""
    return np.abs(lopa[:, 3]) > 139 / 2 * Units.inches


def side_seats(lopa):
    """Seats of the side cabin, both sides."""
    return lopa[(lopa[:, 10] == 1) & is_side(lopa)]


if __name__ == '__main__':
    main()
