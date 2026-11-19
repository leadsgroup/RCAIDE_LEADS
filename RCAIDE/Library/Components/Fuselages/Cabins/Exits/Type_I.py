# RCAIDE/Library/Components/Fuselages/Cabins/Exits/Type_I.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from .Exit import Exit

# ----------------------------------------------------------------------------------------------------------------------
#  Type_I
# ----------------------------------------------------------------------------------------------------------------------
class Type_I(Exit):

    def __defaults__(self):
        """
        Sets default values for a pair of Type I emergency exits (14 CFR 25.807(g): 45 passenger seats per pair).
        """

        self.tag              = 'type_I_exit'
        self.passenger_rating = 45
        self.floor_level      = True
