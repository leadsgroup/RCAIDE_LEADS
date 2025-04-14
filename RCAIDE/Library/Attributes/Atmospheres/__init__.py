# RCAIDE/Library/Attributes/Atmospheres/__init__.py 
# 

"""
RCAIDE Atmospheres module provides atmospheric models for various planetary environments.

The module includes base atmospheric modeling capabilities through the Atmosphere class,
and specific implementations for Earth, although other planets can be added as needed.

The atmospheric models provide essential environmental parameters needed for aircraft
performance calculations, including: Pressure, Temperature, Density, Speed of sound, and Dynamic viscosity.

See Also
--------
RCAIDE.Library.Methods.Aerodynamics : Aerodynamic calculations using atmospheric properties
RCAIDE.Library.Methods.Missions : Mission analysis utilizing atmospheric conditions
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .Atmosphere import Atmosphere

from . import Earth