# RCAIDE/Methods/Power/Fuel_Cell/Sizing/__init__.py
# 

"""
This module provides methods for modeling fuel cell performance using the Larminie-Dicks model. 

It includes functionality for computing voltage, power, and performance characteristics of fuel cells based on 
electrochemical principles. The Larminie-Dicks model is a semi-empirical approach that accounts for activation losses, 
ohmic losses, and concentration losses in fuel cells, making it suitable for system-level design and analysis of fuel 
cell powertrains.

See Also
--------
RCAIDE.Library.Components.Powertrain.Converters.Generic_Fuel_Cell_Stack
RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Common
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from .compute_voltage                import compute_voltage
from .compute_power                  import compute_power
from .compute_power_difference       import compute_power_difference
from .append_fuel_cell_conditions    import append_fuel_cell_conditions
from .compute_fuel_cell_performance  import compute_fuel_cell_performance
