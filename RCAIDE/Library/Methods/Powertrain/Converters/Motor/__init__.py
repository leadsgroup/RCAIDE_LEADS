# RCAIDE/Library/Methods/Powertrain/Converters/Motor/__init__.py

# Created:  Jan 2025, M. Clarke, M. Guidotti

"""
Motor Methods Package

This module contains methods for motor performance analysis, design, and condition handling.
The methods support both DC and PMSM motor modeling within the RCAIDE framework.

See Also
--------
RCAIDE.Library.Components.Powertrain.Converters.DC_Motor
RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .compute_motor_performance             import compute_motor_performance
from .design_DC_motor                       import design_DC_motor
from .append_motor_conditions               import append_motor_conditions 