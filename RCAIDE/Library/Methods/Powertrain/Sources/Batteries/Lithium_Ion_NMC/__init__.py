# RCAIDE/Methods/Powertrain/Sources/Batteries/Lithium_Ion_NMC/__init__.py
# 

"""
This module provides functionality for modeling lithium-ion NMC (nickel manganese cobalt) batteries in powertrains. 
It includes methods for computing NMC cell performance and updating cell age.

See Also
--------
RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC
RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Lithium_Ion_LFP
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from .compute_nmc_cell_performance   import compute_nmc_cell_performance, reuse_stored_nmc_cell_data
from .update_nmc_cell_age            import update_nmc_cell_age