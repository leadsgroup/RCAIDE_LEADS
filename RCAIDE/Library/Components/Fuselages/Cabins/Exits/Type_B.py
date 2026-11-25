# RCAIDE/Library/Components/Fuselages/Cabins/Exits/Type_B.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from .Exit import Exit

# ----------------------------------------------------------------------------------------------------------------------
#  Type_B
# ----------------------------------------------------------------------------------------------------------------------
class Type_B(Exit):

    def __defaults__(self):
        """
        Sets default values for a pair of Type B emergency exits (14 CFR 25.807(g): 75 passenger seats per pair).
        """

        self.tag              = 'type_B_exit'
        self.passenger_rating = 75
        self.floor_level      = True
