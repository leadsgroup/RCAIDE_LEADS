# RCAIDE/Library/Methods/Powertrain/Converters/Ducted_Fan/__init__.py

# Created:  Jan 2025, M. Clarke

"""
Ducted Fan Methods Package

This module contains methods for analyzing and designing ducted fan propulsion systems. The methods support both low and medium-fidelity analysis.

The module provides functionality for:
    - Geometry generation and manipulation
    - Performance analysis and efficiency calculations  
    - DFDC case setup and execution
    - Results processing and data handling

See Also
--------
RCAIDE.Library.Components.Powertrain.Converters.Ducted_Fan
RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan.compute_ducted_fan_performance
RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan.design_ducted_fan
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .compute_ducted_fan_performance     import compute_ducted_fan_performance 
from .append_ducted_fan_conditions       import append_ducted_fan_conditions 
from .design_ducted_fan                  import design_ducted_fan
from .                                   import Performance