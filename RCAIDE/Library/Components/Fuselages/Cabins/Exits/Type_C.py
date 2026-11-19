# RCAIDE/Library/Components/Fuselages/Cabins/Exits/Type_C.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from .Exit import Exit

# ----------------------------------------------------------------------------------------------------------------------
#  Type_C
# ----------------------------------------------------------------------------------------------------------------------
class Type_C(Exit):

    def __defaults__(self):
        """
        Sets default values for a pair of Type C emergency exits (14 CFR 25.807(g): 55 passenger seats per pair).
        """

        self.tag              = 'type_C_exit'
        self.passenger_rating = 55
        self.floor_level      = True
