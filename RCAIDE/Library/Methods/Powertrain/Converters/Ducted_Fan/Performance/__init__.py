# Rotor_Wake provides the functions needed to perform analyses.


"""
Ducted Fan Methods Package

This module contains methods for analyzing and designing ducted fan propulsion systems.
The methods support both low and medium-fidelity analysis.

The module provides functionality for:
    - Geometry generation and manipulation
    - Performance analysis and efficiency calculations  
    - DFDC case setup and execution
    - Results processing and data handling

See Also
--------
RCAIDE.Library.Components.Powertrain.Converters.Ducted_Fan
RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan.Performance.compute_ducted_fan_performance
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# -------------------------------------------------------------------------------------------------------------------- 
from . import Blade_Element_Momentum_Theory
from . import Rankine_Froude_Momentum_Theory




