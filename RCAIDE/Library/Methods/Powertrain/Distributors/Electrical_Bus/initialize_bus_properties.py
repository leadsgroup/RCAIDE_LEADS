#  RCAIDE/Methods/Energy/Distributors/Electrical_Bus/initialize_bus_properties.py
# 
# Created:  Sep 2024, S. Shekar
# Modified: Jan 2025, M. Clarke
#

import RCAIDE
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common          import compute_battery_pack_properties 
from RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Common      import compute_stack_properties

# ----------------------------------------------------------------------------------------------------------------------
#  METHODS
# ---------------------------------------------------------------------------------------------------------------------- 
def initialize_bus_properties(bus,network): 
    return