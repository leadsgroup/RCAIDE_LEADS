# RCAIDE/Library/Missions/__init__.py
# 

"""
This module provides components and utilities for defining and analyzing aircraft missions within the RCAIDE framework.

The Mission module contains three primary submodules:

- Segments: Collection of mission segment types that define different phases of flight
  (e.g., climb, cruise, descent, hover) with specific flight conditions and constraints.

- Common: Shared utilities and data structures used across different mission components,
  including state variables, conditions, and conversion functions.

- Solver: Numerical methods and algorithms for solving mission segment equations
  and propagating the aircraft state through the mission profile.

Together, these components enable the construction of complex mission profiles for
aircraft performance analysis, energy consumption estimation, and mission planning.

See Also
--------
RCAIDE.Analyses.Mission
RCAIDE.Framework.Mission
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from . import Segments
from . import Common
from . import Solver