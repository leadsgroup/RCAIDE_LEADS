# RCAIDE/Methods/Powertrain/Sources/Batteries/Lithium_Ion_LFP/__init__.py
# 

"""
This module provides functionality for modeling lithium-ion LFP (lithium iron phosphate) batteries in powertrains. 
It includes methods for computing LFP cell performance and updating cell age.

See Also
--------
RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_LFP
RCAIDE.Library.Methods.Powertrain.Sources.Batteries
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from .compute_lfp_cell_performance   import * 
from .update_lfp_cell_age            import * 