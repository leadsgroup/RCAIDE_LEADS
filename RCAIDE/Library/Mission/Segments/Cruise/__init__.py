# RCAIDE/Methods/Mission/Segments/Cruise/__init__.py
# 

"""
Collection of cruise segment types for aircraft mission analysis. This module provides various 
cruise profiles including constant Mach, constant speed, loiter patterns, and specialized 
maneuvers at constant altitude with different control parameters (throttle, dynamic pressure, 
pitch rate, etc.).

See Also
--------
RCAIDE.Library.Mission.Segments.Climb
RCAIDE.Library.Mission.Segments.Descent
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
 
from . import Constant_Mach_Constant_Altitude
from . import Constant_Speed_Constant_Altitude
from . import Constant_Mach_Constant_Altitude_Loiter
from . import Constant_Throttle_Constant_Altitude 
from . import Constant_Dynamic_Pressure_Constant_Altitude_Loiter
from . import Constant_Acceleration_Constant_Altitude
from . import Constant_Pitch_Rate_Constant_Altitude
from . import Constant_Dynamic_Pressure_Constant_Altitude
from . import Constant_Speed_Constant_Altitude_Loiter
from . import Curved_Constant_Radius_Constant_Speed_Constant_Altitude
