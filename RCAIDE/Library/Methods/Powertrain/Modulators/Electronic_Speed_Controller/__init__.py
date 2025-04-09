# RCAIDE/Library/Methods/Powertrain/Modulators/Electronic_Speed_Controller/__init__.py
# 

"""
Collection of methods for analyzing Electronic Speed Controller (ESC) performance in electric 
propulsion systems. These devices, ESCs, handle voltage and current modulation for electric motors, 
including throttle response, power conversion, and condition tracking. The module provides 
functionality for computing ESC input/output characteristics and managing electrical state 
variables throughout mission analysis.

See Also
--------
RCAIDE.Library.Components.Powertrain.Converters.DC_Motor
RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .append_esc_conditions  import  append_esc_conditions
from .compute_esc_performance import compute_voltage_out_from_throttle
from .compute_esc_performance import compute_current_in_from_throttle 
