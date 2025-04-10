# RCAIDE/Methods/Energy/Propulsors/Converters/turboshaft/__init__.py
# 

"""
Collection of methods for analyzing turboshaft propulsion systems. These methods handle the design, 
sizing, and performance analysis of turboshaft engines, including power output calculations, core sizing, 
and operational performance evaluation.

The module provides functions for:
    - Computing power output and fuel consumption
    - Sizing engine core components
    - Analyzing design point performance
    - Computing off-design performance characteristics
    - Managing stored engine performance data

See Also
--------
RCAIDE.Library.Components.Powertrain.Converters.Turboshaft_Propulsor
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .append_turboshaft_conditions   import append_turboshaft_conditions
from .compute_power                  import compute_power
from .size_core                      import size_core
from .design_turboshaft              import design_turboshaft
from .compute_turboshaft_performance import compute_turboshaft_performance , reuse_stored_turboshaft_data