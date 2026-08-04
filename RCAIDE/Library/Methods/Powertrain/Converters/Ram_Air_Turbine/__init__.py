# RCAIDE/Library/Methods/Powertrain/Converters/Ram_Air_Turbine/__init__.py

"""
This module provides functionality for modeling ram air turbines (RATs) -- emergency
power generation devices deployed into the freestream airflow -- in powertrains. It
includes methods for computing RAT performance and appending RAT conditions to
simulation results.

See Also
--------
RCAIDE.Library.Methods.Powertrain.Converters
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .append_rat_conditions   import append_rat_conditions                         
from .compute_rat_performance import compute_rat_performance