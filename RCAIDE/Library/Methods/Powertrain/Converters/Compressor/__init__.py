# RCAIDE/Library/Methods/Powertrain/Converters/Compressor/__init__.py
# (c) Copyright 2023 Aerospace Research Community LLC

"""
Collection of methods for computing compressor performance and conditions in propulsion systems. 
This module provides functionality for calculating compressor performance metrics and appending 
relevant thermodynamic conditions to the analysis.

See Also
--------
RCAIDE.Library.Methods.Powertrain.Converters.Turbine
RCAIDE.Library.Methods.Powertrain.Converters.Nozzle
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
        
from .compute_compressor_performance import compute_compressor_performance
from .append_compressor_conditions   import append_compressor_conditions