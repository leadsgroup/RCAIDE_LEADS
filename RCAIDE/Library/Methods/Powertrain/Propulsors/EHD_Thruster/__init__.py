# RCAIDE/Library/Methods/Powertrain/Propulsors/EHD_Thruster/__init__.py
#

"""
Methods for the wire-to-NACA 0010 electrohydrodynamic (EHD) thruster: conditions, per-point performance
(spec 3.3 steps 1-10), reuse for identical thrusters, mass, design sizing and optimizer-input checks.

See Also
--------
RCAIDE.Library.Components.Powertrain.Propulsors.EHD_Thruster
RCAIDE.Library.Methods.Powertrain.Converters.EHD_Electrode_Array
RCAIDE.Library.Methods.Powertrain.Modulators.High_Voltage_Converter
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .append_ehd_thruster_conditions   import append_ehd_thruster_conditions
from .compute_ehd_thruster_performance import compute_ehd_thruster_performance, reuse_stored_ehd_thruster_data
from .compute_ehd_thruster_mass        import compute_ehd_thruster_mass
from .design_ehd_thruster              import design_ehd_thruster
from .check_ehd_optimizer_inputs       import check_ehd_optimizer_inputs
