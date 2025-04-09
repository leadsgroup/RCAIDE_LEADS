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

from .append_ram_conditions   import append_ram_conditions                         
from .compute_ram_performance import compute_ram_performance