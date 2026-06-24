# RCAIDE/Methods/Aeroacoustics/Semi_Empirical/Airframe/__init__.py
# 

""" RCAIDE Package Setup
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from ..compute_aircraft_noise           import airframe_noise 
from .OLD.clean_wing_noise         import clean_wing_noise
from .OLD.landing_gear_noise       import landing_gear_noise
from .OLD.trailing_edge_flap_noise import trailing_edge_flap_noise