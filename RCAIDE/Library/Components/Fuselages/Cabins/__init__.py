"""
Cabin components module for aircraft fuselage interior configurations.

This module provides classes for modeling different types of aircraft cabin 
configurations, including standard passenger cabins and specialized side cabin 
arrangements. These components are used to define interior layouts, passenger 
capacity, and cabin-specific properties within fuselage structures.

See Also
--------
RCAIDE.Library.Components.Fuselages : Parent fuselage components module
RCAIDE.Library.Components.Fuselages.Cabins.Classes : Cabin classification utilities
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .Cabin          import Cabin
from .Side_Cabin     import Side_Cabin
from .               import Classes 