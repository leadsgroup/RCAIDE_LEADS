# RCAIDE/Methods/Energy/Propulsors/Converters/Turbojet/__init__.py
# 

"""
Contains methods for analyzing and designing turbojet propulsion systems. This module provides functionality for computing thrust, 
sizing engine cores, and evaluating turbojet performance characteristics.

See Also
--------
RCAIDE.Library.Components.Powertrain.Propulsors.Converters.Ram
RCAIDE.Library.Components.Powertrain.Propulsors.Converters.Combustor
RCAIDE.Library.Components.Powertrain.Propulsors.Converters.Compressor
RCAIDE.Library.Components.Powertrain.Propulsors.Converters.Turbine
RCAIDE.Library.Components.Powertrain.Propulsors.Converters.Expansion_Nozzle
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .append_turbojet_conditions   import append_turbojet_conditions
from .compute_thurst               import compute_thrust
from .size_core                    import size_core 
from .compute_turbojet_performance import compute_turbojet_performance , reuse_stored_turbojet_data
from .design_turbojet              import design_turbojet