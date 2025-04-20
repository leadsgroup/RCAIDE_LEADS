# RCAIDE/Library/Components/Fuselages/__init__.py

"""
Module containing fuselage components for aircraft design and analysis. This module provides 
different fuselage configurations including traditional tube fuselages, blended wing body 
designs.

See Also
--------
RCAIDE.Library.Components.Airfoils
    Related module for airfoil definitions used in aerodynamic surfaces
RCAIDE.Library.Components.Booms
    Related module for boom components that may connect to fuselage sections
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .Fuselage                   import Fuselage

from . import Segments
from . import Cabins