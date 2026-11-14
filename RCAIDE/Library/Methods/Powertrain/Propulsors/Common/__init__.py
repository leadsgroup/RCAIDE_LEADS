# RCAIDE/Library/Methods/Powertrain/Propulsors/Common/__init__.py
#

"""
Methods shared by propulsors: the conditions every propulsor has, and gas-turbine (Turbofan, Turbojet,
Turboprop) matching and offtake.

See Also
--------
RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan
RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet
RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .compute_gas_generator_shaft_power_offtake import compute_gas_generator_shaft_power_offtake
from .compute_matching_floor                    import compute_matching_floor
from .extend_thrust_below_floor                 import extend_thrust_below_floor
from .append_propulsor_conditions               import append_propulsor_conditions
from .compute_propulsor_moment                 import compute_propulsor_moment
from .pack_fields                              import pack_fields
