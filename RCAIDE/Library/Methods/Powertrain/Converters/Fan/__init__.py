# RCAIDE/Library/Methods/Powertrain/Converters/Fan/__init__.py

"""
This module provides functionality for modeling fans in powertrains. It includes methods for computing 
fan performance and appending fan conditions to simulation results.

See Also
--------
RCAIDE.Library.Methods.Powertrain.Converters
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .append_fan_conditions   import append_fan_conditions                 
from .compute_fan_performance import compute_fan_performance