# RCAIDE/Library/Methods/Geometry/LOPA/compute_exit_passenger_rating.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  compute_exit_passenger_rating
# ----------------------------------------------------------------------------------------------------------------------
def compute_exit_passenger_rating(fuselage):
    '''
    Number of passenger seats the emergency exits allow under 14 CFR 25.807(g): the sum of the ratings of all exit pairs.
    Exits of several cabins at the same station (e.g. the main and side cabins of a blended wing body) share one
    cross-aisle with one pair of exits at the outer walls, so each station counts once, at its highest rating.
    Requires the layout of passenger accommodations to have been computed.

    Parameters
    ----------
    fuselage : Fuselage or Wing
        Component holding the cabins, with its layout_of_passenger_accommodations computed

    Returns
    -------
    passenger_rating : int
        Maximum passenger seats allowed by the exits; compare with fuselage.number_of_seats
    '''
    station_rating = {}
    for station in fuselage.layout_of_passenger_accommodations.exits:
        x = round(float(station.x), 3)   # stations closer than 1 mm are the same cross-aisle
        station_rating[x] = max(station_rating.get(x, 0), station.passenger_rating)
    return int(sum(station_rating.values()))
