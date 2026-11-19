# VnV/Verification/geometry/test_cabin_exits.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Geometry.LOPA import compute_layout_of_passenger_accommodations, compute_exit_passenger_rating

# ----------------------------------------------------------------------------------------------------------------------
#  REFERENCE (hand calculation)
# ----------------------------------------------------------------------------------------------------------------------
# One economy class of 10 rows at 32 in pitch, a Type III pair at mid cabin (0.5) and a Type C pair at the rear (1.0).
#   Type III: row positions are i / 9; 0.5 is equally close to rows 4 and 5, the first is taken (row 4); the exit row
#   and those behind it move aft by 36 - 32 = 4 in (cabin default exit-row pitch). Row 4 center = 4 x 32 + 4 + 16 = 148 in
#   door row: 36 in long (cabin default), behind the last seat row: 9 x 32 + 4 + 36 + 16 = 344 in
#   passenger rating, 14 CFR 25.807(g): Type C 55 + Type III 35 = 90 per side pair
DOOR_X      = 344 * Units.inches
OVERWING_X  = 148 * Units.inches
RATING      = 90


# ----------------------------------------------------------------------------------------------------------------------
#  MAIN
# ----------------------------------------------------------------------------------------------------------------------
def main():
    check_exit_stations()
    check_exit_rows_in_layout()
    check_passenger_rating()
    return


def check_exit_stations():
    """Each exit pair is reported with its tag, type and hand-calculated station."""
    exits = {station.tag: station for station in layout().layout_of_passenger_accommodations.exits}
    assert set(exits) == {'rear_exit', 'overwing_exit'}
    assert exits['rear_exit'].exit_type == 'Type_C' and exits['rear_exit'].floor_level
    assert exits['overwing_exit'].exit_type == 'Type_III' and not exits['overwing_exit'].floor_level
    assert np.isclose(exits['rear_exit'].x, DOOR_X)
    assert np.isclose(exits['overwing_exit'].x, OVERWING_X)
    return


def check_exit_rows_in_layout():
    """The floor-level exit becomes one door row and the Type III exit one emergency-exit seat row."""
    lopa       = np.asarray(layout().layout_of_passenger_accommodations.object_coordinates)
    door_x     = np.unique(lopa[lopa[:, 13] == 1, 2])
    overwing_x = np.unique(lopa[lopa[:, 11] == 1, 2])
    assert len(door_x) == 1 and np.isclose(door_x[0], DOOR_X)
    assert len(overwing_x) == 1 and np.isclose(overwing_x[0], OVERWING_X)
    return


def check_passenger_rating():
    """The exits allow the sum of their 14 CFR 25.807(g) ratings."""
    assert compute_exit_passenger_rating(layout()) == RATING
    return


# ----------------------------------------------------------------------------------------------------------------------
#  HELPERS
# ----------------------------------------------------------------------------------------------------------------------
def layout():
    """Fuselage with one economy class, a Type III pair at mid cabin and a Type C pair at the rear."""
    fuselage = RCAIDE.Library.Components.Fuselages.Fuselage()
    cabin    = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
    economy  = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy()
    economy.number_of_seats_abrest = 6
    economy.number_of_rows         = 10

    door                    = RCAIDE.Library.Components.Fuselages.Cabins.Exits.Type_C()
    door.tag                = 'rear_exit'
    door.percent_x_location = 1.0
    economy.append_exit(door)

    overwing                    = RCAIDE.Library.Components.Fuselages.Cabins.Exits.Type_III()
    overwing.tag                = 'overwing_exit'
    overwing.percent_x_location = 0.5
    economy.append_exit(overwing)

    cabin.append_cabin_class(economy)
    fuselage.append_cabin(cabin)
    compute_layout_of_passenger_accommodations(fuselage)
    return fuselage


if __name__ == '__main__':
    main()
