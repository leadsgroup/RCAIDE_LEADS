## @defgroup Library-Plots-Energy 
# RCAIDE/Library/Plots/Performance/__init__.py
# 

"""
Energy plotting module providing visualization tools for energy-related components and analysis.

This module contains functions for plotting various energy-related metrics and characteristics
including battery performance, propulsion system efficiency, and
fuel consumption patterns.
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
 
from .plot_battery_module_conditions            import plot_battery_module_conditions
from .plot_battery_cell_conditions              import plot_battery_cell_conditions
from .plot_battery_degradation                  import plot_battery_degradation 
from .plot_battery_temperature                  import plot_battery_temperature
from .plot_battery_module_C_rates               import plot_battery_module_C_rates
from .plot_battery_pack_conditions              import plot_battery_pack_conditions
from .plot_battery_ragone_diagram               import plot_battery_ragone_diagram 
from .plot_motor_conditions                     import plot_motor_conditions 
from .plot_altitude_sfc_weight                  import plot_altitude_sfc_weight
from .plot_propulsor_throttles                  import plot_propulsor_throttles
from .plot_payload_range_diagram                import plot_payload_range_diagram
from .plot_V_n_diagram                          import plot_V_n_Diagram