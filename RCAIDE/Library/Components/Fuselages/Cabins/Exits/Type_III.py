# RCAIDE/Library/Components/Fuselages/Cabins/Exits/Type_III.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from .Exit import Exit

# ----------------------------------------------------------------------------------------------------------------------
#  Type_III
# ----------------------------------------------------------------------------------------------------------------------
class Type_III(Exit):

    def __defaults__(self):
        """
        Sets default values for a pair of Type III emergency exits (14 CFR 25.807(g): 35 passenger seats per pair).
        """

        self.tag              = 'type_III_exit'
        self.passenger_rating = 35
        self.floor_level      = False
