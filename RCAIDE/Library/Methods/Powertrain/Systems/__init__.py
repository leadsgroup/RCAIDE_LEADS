# RCAIDE/Methods/Energy/Propulsion/Converters/Common/__init__.py
# 

"""
This module provides functionality for setting up and managing systems that draw power from the powertrain system, such as avionics and payloads. 
It includes methods for configuring operating conditions and appending avionics and payload conditions to simulation results.

See Also
--------
RCAIDE.Library.Methods.Powertrain.Converters
RCAIDE.Library.Methods.Powertrain.Sources
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .setup_operating_conditions             import setup_operating_conditions
from .append_avionics_conditions             import append_avionics_conditions
from .append_payload_conditions              import append_payload_conditions 