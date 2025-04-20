# RCAIDE/Library/Components/Fuselages/__init__.py

"""
Module containing fuselage segments for aircraft design and analysis.

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

from .Segment                    import Segment
from .Circle_Segment             import Circle_Segment
from .Ellipse_Segment            import Ellipse_Segment
from .Super_Ellipse_Segment      import Super_Ellipse_Segment
from .Rounded_Rectangle_Segment  import Rounded_Rectangle_Segment