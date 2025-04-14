# RCAIDE/Methods/Mission/Segments/Transition/__init__.py
# 

"""
Collection of mission segment types that handle aircraft transition maneuvers, including constant acceleration transitions with various climb and pitch configurations.

See Also
--------
RCAIDE.Library.Mission.Segments.Cruise
RCAIDE.Library.Mission.Segments.Climb
RCAIDE.Library.Mission.Segments.Descent
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
 
from . import Constant_Acceleration_Constant_Pitchrate_Constant_Altitude
from . import Constant_Acceleration_Constant_Angle_Linear_Climb 