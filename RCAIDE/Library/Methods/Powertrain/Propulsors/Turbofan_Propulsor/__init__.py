# RCAIDE/Methods/Energy/Propulsors/Converters/Turbofan/__init__.py
# 

"""
Collection of methods for modeling and analyzing turbofan propulsion systems. This module provides functionality for designing, 
sizing, and computing performance characteristics of turbofan engines including thrust calculations and core sizing.
 
See Also
 --------
RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan
RCAIDE.Framework.Analysis.Propulsion
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
 
from .append_turbofan_conditions                                 import append_turbofan_conditions 
from .compute_thurst                                             import compute_thrust
from .size_core                                                  import size_core 
from .compute_turbofan_performance                               import compute_turbofan_performance 
from .compute_turbofan_performance                               import reuse_stored_turbofan_data
from .design_turbofan                                            import design_turbofan    
