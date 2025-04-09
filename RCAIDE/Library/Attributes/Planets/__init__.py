# RCAIDE/Library/Attributes/Planets/__init__.py
# 

"""
This module provides planetary environment definitions for RCAIDE simulations. It includes 
base classes and specific implementation for Earth.

The module supports atmospheric calculations, gravitational effects, and other planetary 
environmental factors needed for aerospace vehicle analysis and simulation.

See Also
--------
RCAIDE.Library.Attributes.Atmospheres : Detailed atmospheric models
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .Planet import Planet
from .Earth  import Earth