# RCAIDE/Library/Components/Fuselages/Cabins/Exits/Exit.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Library.Components import Component

# ----------------------------------------------------------------------------------------------------------------------
#  Exit
# ----------------------------------------------------------------------------------------------------------------------
class Exit(Component):

    def __defaults__(self):
        """
        Sets default values for a pair of emergency exits, one on each side of the cabin.

        The passenger rating is the number of passenger seats a pair of exits of this type allows under
        14 CFR 25.807(g). Floor-level exits are placed as a door row across the cabin; other exits are placed
        at an emergency-exit seat row. The location is a fraction of the cabin class length.
        """

        self.tag                = 'exit'
        self.percent_x_location = 0.0
        self.passenger_rating   = 0
        self.floor_level        = True
