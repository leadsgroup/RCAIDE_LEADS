# RCAIDE_LEADS/RCAIDE/Library/Methods/Mass_Properties/Weight_Buildups/Conventional/__init__.py
# 

"""
This module initializes the Conventional weight buildup methods for the RCAIDE library. Conventional aircraft are defined 
as aircraft that use conventional propulsion systems, such as turbofans or turbojets. It imports various submodules
that handle different types of aircraft weight calculations, such as Blended Wing Body (BWB), General Aviation, and 
Transport aircraft. The Common module provides shared utilities and functions used across these submodules.

See Also
--------
RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Electric
RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Hybrid
RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Hydrogen
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from . import BWB
from . import General_Aviation
from . import Transport
from . import Common