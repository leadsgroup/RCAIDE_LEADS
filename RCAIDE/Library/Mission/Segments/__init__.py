# RCAIDE/Library/Mission/Segments/__init__.py
# 

"""
This module initializes the RCAIDE package by importing various mission segment modules. 
These segments include Cruise, Climb, Descent, Ground, Vertical Flight, Single Point, Transition, and Untrimmed. 
Each segment represents a different phase of a mission and provides specific functionalities related to that phase.

See Also
--------
RCAIDE.Library.Methods
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from . import Cruise
from . import Climb
from . import Descent
from . import Ground
from . import Vertical_Flight
from . import Single_Point
from . import Transition
from . import Untrimmed