# RCAIDE/Library/Components/Fuselages/Cabins/Exits/Type_A.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from .Exit import Exit

# ----------------------------------------------------------------------------------------------------------------------
#  Type_A
# ----------------------------------------------------------------------------------------------------------------------
class Type_A(Exit):

    def __defaults__(self):
        """
        Sets default values for a pair of Type A emergency exits (14 CFR 25.807(g): 110 passenger seats per pair).
        """

        self.tag              = 'type_A_exit'
        self.passenger_rating = 110
        self.floor_level      = True
