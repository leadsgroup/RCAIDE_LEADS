## @defgroup Library-Plots-Energy 
# RCAIDE/Library/Plots/Energy/__init__.py
# 

"""
Energy plotting module providing visualization tools for energy-related components and analysis.

This module contains functions for plotting various energy-related metrics and characteristics
including battery performance, solar network conditions, propulsion system efficiency, and
fuel consumption patterns.
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .plot_solar_network_conditions             import plot_solar_network_conditions 
from .plot_battery_module_conditions            import plot_battery_module_conditions
from .plot_battery_cell_conditions              import plot_battery_cell_conditions
from .plot_battery_degradation                  import plot_battery_degradation 
from .plot_battery_temperature                  import plot_battery_temperature
from .plot_battery_module_C_rates               import plot_battery_module_C_rates 
from .plot_battery_ragone_diagram               import plot_battery_ragone_diagram 
from .plot_electric_propulsor_efficiencies      import plot_electric_propulsor_efficiencies
from .plot_fuel_consumption                     import plot_fuel_consumption
from .plot_altitude_sfc_weight                  import plot_altitude_sfc_weight
from .plot_propulsor_throttles                  import plot_propulsor_throttles               