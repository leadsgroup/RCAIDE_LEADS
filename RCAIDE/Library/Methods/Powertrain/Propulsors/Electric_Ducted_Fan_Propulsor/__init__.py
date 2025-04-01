# RCAIDE/Methods/Energy/Propulsors/Electric_Ducted_Fan_Propulsor/__init__.py
# 

"""
Methods for analyzing electric ducted fan (EDF) propulsion systems. This module provides functionality for 
computing performance, managing operating conditions, and handling system residuals for electric ducted fan 
propulsors.

See Also
--------
RCAIDE.Library.Methods.Powertrain.Propulsors.Common
RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Ducted_Fan
RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .compute_electric_ducted_fan_performance  import compute_electric_ducted_fan_performance
from .append_electric_ducted_fan_conditions    import append_electric_ducted_fan_conditions
from .pack_electric_ducted_fan_residuals       import pack_electric_ducted_fan_residuals
from .unpack_electric_ducted_fan_unknowns      import unpack_electric_ducted_fan_unknowns