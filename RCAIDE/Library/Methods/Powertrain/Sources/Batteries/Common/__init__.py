"""
This module provides common functionality for modeling battery systems in powertrains. It includes methods for 
battery sizing, performance calculation, condition management, and property computation that are shared across
different battery chemistries.

The module contains functions for:
    - Appending and managing battery conditions during mission segments
    - Finding battery properties using Ragone curves
    - Calculating power and energy characteristics
    - Computing mass changes for metal-air batteries
    - Sizing battery modules based on mass or energy/power requirements

See Also
--------
RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_LFP
RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC
RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Aluminum_Air
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

from .append_battery_conditions               import append_battery_conditions, append_battery_segment_conditions
from .find_ragone_properties                  import find_ragone_properties
from .find_specific_power                     import find_specific_power
from .find_mass_gain_rate                     import find_mass_gain_rate
from .find_total_mass_gain                    import find_total_mass_gain
from .size_module_from_mass                   import size_module_from_mass
from .size_module_from_energy_and_power       import size_module_from_energy_and_power
from .compute_module_properties               import compute_module_properties 