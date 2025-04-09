# RCAIDE/Library/Methods/Powertrain/Converters/Compression_Nozzle/__init__.py

"""
This module provides functionality for modeling compression nozzles in powertrains. It includes methods for computing 
compression nozzle performance and appending compression nozzle conditions to simulation results.

See Also
--------
RCAIDE.Library.Methods.Powertrain.Converters
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
         
from .compute_compression_nozzle_performance import compute_compression_nozzle_performance
from .append_compression_nozzle_conditions   import append_compression_nozzle_conditions 