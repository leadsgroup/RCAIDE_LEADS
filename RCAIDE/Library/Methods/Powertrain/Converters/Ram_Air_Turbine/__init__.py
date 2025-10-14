# RCAIDE/Library/Methods/Powertrain/Converters/Ram/__init__.py

"""
This module provides functionality for modeling ram air compression in powertrains. It includes methods for computing 
ram compression performance and appending ram conditions to simulation results.

See Also
--------
RCAIDE.Library.Methods.Powertrain.Converters
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .append_rat_conditions   import append_rat_conditions                         
from .compute_rat_performance import compute_rat_performance