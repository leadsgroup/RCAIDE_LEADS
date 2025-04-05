# RCAIDE/Methods/Energy/Propulsors/Converters/turboshaft/__init__.py
# 

"""
Contains methods for analyzing and designing turboprop propulsion systems. This module provides a comprehensive set of tools 
for turboprop engine analysis, combining gas turbine core performance with propeller aerodynamics. The methods handle both 
design and off-design conditions, supporting complete propulsion system analysis across flight regimes.

The module includes functionality for:
    - Computing thrust and power output
    - Sizing gas turbine core components
    - Evaluating combined engine-propeller performance
    - Analyzing propeller aerodynamic characteristics
    - Designing complete turboprop systems from specifications
    - Computing fuel consumption and efficiency metrics

See Also
--------
RCAIDE.Library.Methods.Powertrain.Propulsors.Converters.Ram
RCAIDE.Library.Methods.Powertrain.Propulsors.Converters.Combustor
RCAIDE.Library.Methods.Powertrain.Propulsors.Converters.Compressor
RCAIDE.Library.Methods.Powertrain.Propulsors.Converters.Turbine
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .append_turboprop_conditions   import append_turboprop_conditions
from .size_core                     import size_core
from .design_turboprop              import design_turboprop
from .compute_thrust                import compute_thrust
from .compute_turboprop_performance import compute_turboprop_performance , reuse_stored_turboprop_data