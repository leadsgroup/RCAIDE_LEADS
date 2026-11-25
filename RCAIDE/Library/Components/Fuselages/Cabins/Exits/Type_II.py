# RCAIDE/Library/Components/Fuselages/Cabins/Exits/Type_II.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from .Exit import Exit

# ----------------------------------------------------------------------------------------------------------------------
#  Type_II
# ----------------------------------------------------------------------------------------------------------------------
class Type_II(Exit):

    def __defaults__(self):
        """
        Sets default values for a pair of Type II emergency exits (14 CFR 25.807(g): 40 passenger seats per pair).
        """

        self.tag              = 'type_II_exit'
        self.passenger_rating = 40
        self.floor_level      = False
