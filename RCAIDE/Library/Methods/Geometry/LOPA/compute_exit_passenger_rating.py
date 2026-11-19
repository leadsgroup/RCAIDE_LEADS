# RCAIDE/Library/Methods/Geometry/LOPA/compute_exit_passenger_rating.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  compute_exit_passenger_rating
# ----------------------------------------------------------------------------------------------------------------------
def compute_exit_passenger_rating(fuselage):
    '''
    Number of passenger seats the emergency exits allow under 14 CFR 25.807(g): the sum of the ratings of all exit pairs.
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
    exits = fuselage.layout_of_passenger_accommodations.exits
    return int(sum(station.passenger_rating for station in exits))
