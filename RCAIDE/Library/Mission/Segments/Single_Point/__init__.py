# RCAIDE/Methods/Mission/Segments/Single_Point/__init__.py
# 

"""
Collection of single-point analysis segments for aircraft mission simulation. This module 
provides segments for analyzing aircraft performance at specific flight conditions with 
various control parameters including speed, altitude, and throttle settings. Includes 
specialized segments for AVL (Athena Vortex Lattice) trimmed analysis.

See Also
--------
RCAIDE.Library.Mission.Segments.Climb
RCAIDE.Library.Mission.Segments.Cruise
RCAIDE.Library.Mission.Segments.Descent
RCAIDE.Library.Mission.Segments.Ground
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from . import Set_Speed_Set_Altitude
from . import Set_Speed_Set_Throttle 
from . import Set_Speed_Set_Altitude_AVL_Trimmed
from . import Set_Speed_Set_Altitude_No_Propulsion