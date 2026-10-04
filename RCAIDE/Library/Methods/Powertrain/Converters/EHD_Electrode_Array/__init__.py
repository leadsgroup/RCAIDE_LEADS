# RCAIDE/Library/Methods/Powertrain/Converters/EHD_Electrode_Array/__init__.py
#

"""
Methods for the wire-to-NACA 0010 electrohydrodynamic (EHD) electrode array: Peek inception (R19, R20),
1-D space-charge thrust and power (R21, R22, spec 2.4 derived equations), electrode drag (R24), mass and
input checks.

See Also
--------
RCAIDE.Library.Components.Powertrain.Converters.EHD_Electrode_Array
RCAIDE.Library.Methods.Powertrain.Propulsors.EHD_Thruster
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .compute_peek_inception                 import compute_relative_air_density, compute_peek_inception_field, compute_inception_voltage
from .compute_space_charge_performance       import compute_dimensionless_current, compute_current_density, compute_unit_thrust_and_power
from .compute_electrode_drag                 import compute_collector_drag_coefficient, compute_electrode_drag
from .compute_electrode_array_mass           import compute_electrode_array_mass
from .check_ehd_electrode_array_inputs       import check_ehd_electrode_array_inputs, check_fixed_value, check_spacing_correction
from .append_ehd_electrode_array_conditions  import append_ehd_electrode_array_conditions
