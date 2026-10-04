# RCAIDE/Library/Methods/Powertrain/Modulators/High_Voltage_Converter/__init__.py
#

"""
Methods for the fixed-efficiency high-voltage power converter (HVPC) that feeds an EHD thruster.

See Also
--------
RCAIDE.Library.Components.Powertrain.Modulators.High_Voltage_Converter
RCAIDE.Library.Methods.Powertrain.Propulsors.EHD_Thruster
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .append_hvpc_conditions   import append_hvpc_conditions
from .compute_hvpc_performance import compute_hvpc_performance
from .compute_hvpc_mass        import compute_hvpc_mass
