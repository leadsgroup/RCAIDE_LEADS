# RCAIDE/Methods/Mission/Segments/Descent/__init__.py
# 

"""
Collection of descent segment types for aircraft mission analysis. This module provides various 
descent profiles including constant speed, linear Mach, constant angle, and different airspeed 
measurement methods (EAS, CAS) combined with constant rate descents.

See Also
--------
RCAIDE.Library.Mission.Segments.Climb
RCAIDE.Library.Mission.Segments.Cruise
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from . import Constant_Speed_Constant_Rate
from . import Linear_Mach_Constant_Rate
from . import Linear_Speed_Constant_Rate
from . import Constant_Speed_Constant_Angle 
from . import Constant_EAS_Constant_Rate
from . import Constant_CAS_Constant_Rate