# RCAIDE/Methods/Energy/Propulsors/ICE_Propulsor/__init__.py
# 

"""
Methods for modeling and analyzing internal combustion engine propulsors.

This module provides functions for designing, analyzing, and simulating internal combustion 
engines in aircraft propulsion systems. It includes methods for computing engine performance, 
handling residuals and unknowns in numerical solvers, and managing engine operating conditions.

See Also
--------
RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop
RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet
RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .append_internal_combustion_engine_residual_and_unknown import append_internal_combustion_engine_residual_and_unknown
from .compute_internal_combustion_engine_performance         import compute_internal_combustion_engine_performance
from .append_internal_combustion_engine_conditions           import append_internal_combustion_engine_conditions
from .unpack_internal_combustion_engine_unknowns             import unpack_internal_combustion_engine_unknowns
from .pack_internal_combustion_engine_residuals              import pack_internal_combustion_engine_residuals
from .design_internal_combustion_engine                      import design_internal_combustion_engine


