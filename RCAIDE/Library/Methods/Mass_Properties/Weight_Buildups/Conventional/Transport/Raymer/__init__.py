# RCAIDE/Library/Methods/Mass_Properties/Weight_Buildups/Conventional/Transport/Raymer/__init__.py
# 

"""
Collection of aircraft weight estimation methods based on Raymer's empirical correlations for a transport aircraft. 
This module provides component weight estimation functions including wing, tail, fuselage, landing gear, 
and systems weights. The correlations are primarily derived from historical aircraft data as presented in 
Raymer's "Aircraft Design: A Conceptual Approach".

See Also
--------
RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.FLOPS
RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.General_Aviation
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .compute_operating_empty_weight import compute_operating_empty_weight
from .compute_main_wing_weight import compute_main_wing_weight
from .compute_horizontal_tail_weight import compute_horizontal_tail_weight
from .compute_vertical_tail_weight import compute_vertical_tail_weight
from .compute_fuselage_weight import compute_fuselage_weight
from .compute_landing_gear_weight import compute_landing_gear_weight
from .compute_propulsion_system_weight import *
from .compute_systems_weight import *