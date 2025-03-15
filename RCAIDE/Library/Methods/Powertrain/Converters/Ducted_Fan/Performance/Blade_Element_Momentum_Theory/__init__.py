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

from .purge_files                        import purge_files
from .read_results                       import read_results
from .run_dfdc_analysis                  import run_dfdc_analysis
from .translate_conditions_to_dfdc_cases import translate_conditions_to_dfdc_cases  
from .write_geometry                     import write_geometry
from .write_input_deck                   import write_input_deck 