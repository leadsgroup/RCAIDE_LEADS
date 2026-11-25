# RCAIDE/Library/Components/Fuselages/Cabins/Exits/Type_IV.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from .Exit import Exit

# ----------------------------------------------------------------------------------------------------------------------
#  Type_IV
# ----------------------------------------------------------------------------------------------------------------------
class Type_IV(Exit):

    def __defaults__(self):
        """
        Sets default values for a pair of Type IV emergency exits (14 CFR 25.807(g): 9 passenger seats per pair).
        """

        self.tag              = 'type_IV_exit'
        self.passenger_rating = 9
        self.floor_level      = False
