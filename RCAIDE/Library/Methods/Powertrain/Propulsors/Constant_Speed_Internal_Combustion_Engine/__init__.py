# RCAIDE/Methods/Energy/Propulsors/Constant_Speed_ICE_Propulsor/__init__.py
# 

"""
Methods for modeling and analyzing constant speed internal combustion engine propulsors.

This module provides functions for designing, analyzing, and simulating internal combustion 
engines that operate at a constant speed in aircraft propulsion systems. These methods are 
particularly useful for modeling generators and auxiliary power units where the engine 
operates at a fixed RPM regardless of the aircraft's flight condition.

The module includes methods for computing engine performance, appending engine conditions 
to mission segments, and designing the engine-propeller combination for optimal performance.

See Also
--------
RCAIDE.Library.Methods.Powertrain.Propulsors.Internal_Combustion_Engine
RCAIDE.Library.Methods.Powertrain.Converters.Engine
RCAIDE.Library.Methods.Powertrain.Converters.Rotor
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .compute_constant_speed_internal_combustion_engine_performance  import compute_constant_speed_internal_combustion_engine_performance
from .append_constant_speed_internal_combustion_engine_conditions    import append_constant_speed_internal_combustion_engine_conditions 
from .design_constant_speed_internal_combustion_engine               import design_constant_speed_internal_combustion_engine
