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

from .Segment                   import Segment, Segment_Container
from .Blended_Wing_Body_Fuselage_Segment import  Blended_Wing_Body_Fuselage_Segment
