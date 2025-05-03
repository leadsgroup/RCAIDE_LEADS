# RCAIDE/Library/Components/Wings/__init__.py

"""
Module containing wing and lifting surface components for aircraft design and analysis. 
This module provides classes for various wing types including main wings, vertical and 
horizontal tails, and all-moving surfaces.

The module also includes a Control_Surfaces subpackage for various control surface types.

See Also
--------
RCAIDE.Library.Components.Airfoils
    Related module for airfoil definitions used in wing design
RCAIDE.Library.Components.Fuselages
    Related module for fuselage components that interface with wings
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .Wing                      import Wing
from .Main_Wing                 import Main_Wing
from .Blended_Wing_Body         import Blended_Wing_Body
from .Vertical_Tail             import Vertical_Tail
from .Horizontal_Tail           import Horizontal_Tail
from .All_Moving_Surface        import All_Moving_Surface
from .Stabilator                import Stabilator
from .Vertical_Tail_All_Moving  import Vertical_Tail_All_Moving

# packages
from . import Control_Surfaces
from . import Segments