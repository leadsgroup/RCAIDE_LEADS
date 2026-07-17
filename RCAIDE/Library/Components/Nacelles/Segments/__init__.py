# RCAIDE/Library/Components/Nacelles/Segments/__init__.py

"""
Module containing nacelle segments for aircraft propulsion system integration.

See Also
--------
RCAIDE.Library.Components.Fuselages
    Related module for nacelle components that may interface with engine nacelles
RCAIDE.Library.Components.Landing_Gear
    Related module for landing gear design which may affect nacelle ground clearance
RCAIDE.Library.Components.Airfoils
    Related module for airfoil definitions that may influence nacelle-wing integration
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

from .Segment                    import Segment 
from .Circle_Segment             import Circle_Segment
from .Ellipse_Segment            import Ellipse_Segment
from .Super_Ellipse_Segment      import Super_Ellipse_Segment
from .Rounded_Rectangle_Segment  import Rounded_Rectangle_Segment