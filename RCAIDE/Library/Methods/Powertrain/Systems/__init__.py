# RCAIDE/Methods/Energy/Propulsion/Converters/Common/__init__.py
# 

"""
This module provides functionality for setting up and managing systems that draw power from the powertrain system, such as avionics and payloads. 
It includes methods for configuring operating conditions and appending avionics and systems conditions to simulation results.

See Also
--------
RCAIDE.Library.Methods.Powertrain.Converters
RCAIDE.Library.Methods.Powertrain.Sources
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .append_systems_conditions                 import append_systems_conditions 
from .compute_avionics_power_draw               import compute_avionics_power_draw
from .compute_flight_controls_power_draw        import compute_flight_controls_power_draw
from .compute_systems_power_draw                import compute_systems_power_draw
from .compute_ecs_power_draw                    import compute_ecs_power_draw
from .compute_hydraulics_power_draw             import compute_hydraulics_power_draw
from .compute_cabin_loads_power_draw            import compute_cabin_loads_power_draw
from .compute_ice_protection_power_draw         import compute_ice_protection_power_draw