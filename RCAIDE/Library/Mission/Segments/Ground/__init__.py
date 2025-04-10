# RCAIDE/Methods/Mission/Segments/Ground/__init__.py
# 

"""
Collection of ground operation segments for aircraft mission analysis. This module provides 
segments for ground-based operations including takeoff, landing, and battery charging or 
discharging while the aircraft is stationary.

See Also
--------
RCAIDE.Library.Mission.Segments.Climb
RCAIDE.Library.Mission.Segments.Cruise
RCAIDE.Library.Mission.Segments.Descent
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
 
from . import Takeoff
from . import Landing
from . import Battery_Charge_Discharge