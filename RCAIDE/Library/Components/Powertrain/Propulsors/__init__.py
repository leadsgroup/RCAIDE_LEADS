# RCAIDE/Library/Components/Propulsors/__init__.py
# 
# 
"""
Collection of propulsion system components for aircraft modeling and simulation.

This module provides various propulsor classes for different types of aircraft 
propulsion systems, including:

- Electric propulsion (Electric_Rotor, Electric_Ducted_Fan)
- Internal combustion engine propulsion (ICE_Propeller, Constant_Speed_ICE_Propeller)
- Gas turbine propulsion (Turbofan, Turbojet, Turboshaft, Turboprop)
- Power conversion components (Converters)

Notes
-----
Each propulsor class inherits from the base Propulsor class and implements 
specific methods for performance calculation, operating condition management, 
and system state tracking.

The module structure allows for easy extension with new propulsion system types
while maintaining consistent interfaces across all implementations.

See Also
--------
RCAIDE.Library.Components.Energy
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .Propulsor                                  import Propulsor
from .Electric_Rotor                             import Electric_Rotor
from .Electric_Ducted_Fan                        import Electric_Ducted_Fan
from .Internal_Combustion_Engine                 import Internal_Combustion_Engine
from .Turbofan                                   import Turbofan
from .Turbojet                                   import Turbojet
from .Turboprop                                  import Turboprop
from .Constant_Speed_Internal_Combustion_Engine  import Constant_Speed_Internal_Combustion_Engine
