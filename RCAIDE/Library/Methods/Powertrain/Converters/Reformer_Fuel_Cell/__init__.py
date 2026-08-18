# RCAIDE/Library/Methods/Powertrain/Converters/Reformer_Fuel_Cell/__init__.py
#

"""
This module provides methods for modeling and analyzing reformer/fuel-cell composite
systems in aircraft powertrains -- a fuel cell that draws its hydrogen from an onboard
reformer converting a hydrocarbon fuel, rather than from a hydrogen fuel tank. It includes
functionality for performance computation and condition management.

See Also
--------
RCAIDE.Library.Components.Powertrain.Converters.Reformer_Fuel_Cell
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .append_reformer_fuel_cell_conditions   import append_reformer_fuel_cell_conditions
from .compute_reformer_fuel_cell_performance import compute_reformer_fuel_cell_performance
