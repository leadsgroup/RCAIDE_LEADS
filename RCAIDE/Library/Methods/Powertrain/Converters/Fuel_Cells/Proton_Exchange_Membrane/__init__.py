# RCAIDE/Methods/Power/Fuel_Cell/Sizing/__init__.py
# 

"""
This module provides methods for modeling and analyzing Proton Exchange Membrane (PEM) fuel cells 
in aircraft powertrain systems. 

It includes functionality for computing performance characteristics and appending operating 
conditions during mission analysis. PEM fuel cells are characterized by their use of a polymer 
electrolyte membrane that conducts protons while being impermeable to gases. These methods 
implement specialized models that account for the unique electrochemical and thermal characteristics 
of PEM fuel cells.

See Also
--------
RCAIDE.Library.Components.Energy.Converters.Fuel_Cell_Stack
RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Common
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .append_fuel_cell_conditions   import *
from .compute_fuel_cell_performance import *
