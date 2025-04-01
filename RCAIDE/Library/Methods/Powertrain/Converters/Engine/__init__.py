# RCAIDE/Library/Methods/Powertrain/Converters/Engine/__init__.py
# (c) Copyright 2023 Aerospace Research Community LLC

"""
Collection of methods for engine performance calculations and condition management. This module provides utilities for 
handling engine throttle settings, power calculations, and operating conditions for propulsion system analysis.

See Also
--------
RCAIDE.Library.Methods.Powertrain.Converters.Turbine
RCAIDE.Library.Methods.Powertrain.Converters.Motor
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .append_engine_conditions           import append_engine_conditions         
from .compute_throttle_from_power        import compute_throttle_from_power
from .compute_power_from_throttle        import compute_power_from_throttle 