# RCAIDE/Methods/Energy/Propulsors/Converters/Turbojet/__init__.py
# 

"""
Contains methods for analyzing and designing turbojet propulsion systems. This module provides functionality for computing thrust, 
sizing engine cores, and evaluating turbojet performance characteristics.

See Also
--------
RCAIDE.Library.Components.Powertrain.Converters.Ram
RCAIDE.Library.Components.Powertrain.Converters.Combustor
RCAIDE.Library.Components.Powertrain.Converters.Compressor
RCAIDE.Library.Components.Powertrain.Converters.Turbine
RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .append_turbojet_conditions   import append_turbojet_conditions
from .compute_thurst               import compute_thrust
from .size_core                    import size_core 
from .compute_turbojet_performance import compute_turbojet_performance , reuse_stored_turbojet_data
from .compute_turbojet_performance import compute_turbojet_performance_offdesign
from .design_turbojet              import design_turbojet
from .design_turbojet_offdesign_matching import design_turbojet_offdesign_matching
from .Turbojet_OffDesign_Matching  import OffDesignMatchingError
from .Turbojet_OffDesign_Matching  import solve_turbojet_offdesign
from .Turbojet_OffDesign_Matching  import solve_turbojet_offdesign_robust
from .generate_turbojet_deck import generate_turbojet_deck
