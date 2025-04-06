# RCAIDE/Methods/Energy/Propulsors/Converters/Turboelectric_Generator/__init__.py
# 

"""
This module provides methods for modeling and analyzing turboelectric generator systems in aircraft powertrains. It includes functionality for design, performance computation, and condition management of turboelectric generators.

See Also
--------
RCAIDE.Library.Components.Energy.Converters.Turboelectric_Generator
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .append_turboelectric_generator_conditions   import append_turboelectric_generator_conditions 
from .compute_turboelectric_generator_performance import compute_turboelectric_generator_performance , reuse_stored_turboelectric_generator_data
from .design_turboelectric_generator              import design_turboelectric_generator