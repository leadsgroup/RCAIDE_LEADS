# RCAIDE/Library/Components/Nacelles/__init__.py

"""
Module containing nacelle components for aircraft propulsion system integration. This module 
provides different nacelle configurations including stack nacelles and bodies of revolution 
for various engine installations.

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

from .Nacelle                    import Nacelle 
from .Stack_Nacelle              import Stack_Nacelle
from .Body_of_Revolution_Nacelle import Body_of_Revolution_Nacelle

from . import Segments