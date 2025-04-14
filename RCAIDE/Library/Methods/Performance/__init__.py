# RCAIDE/Methods/Performance/__init__.py
# 

"""
Methods for analyzing vehicle performance characteristics including aerodynamics, propulsion, 
and flight mechanics. 
 
This module provides functions for estimating key performance metrics 
such as take-off and landing distances, stall speeds, payload-range capabilities, and flight 
envelope characteristics.
 
See Also
--------
RCAIDE.Library.Mission
RCAIDE.Library.Methods.Aerodynamics
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .aircraft_aerodynamic_analysis     import aircraft_aerodynamic_analysis
from .estimate_take_off_field_length    import estimate_take_off_field_length
from .estimate_stall_speed              import estimate_stall_speed
from .compute_payload_range_diagram     import compute_payload_range_diagram
from .estimate_landing_field_length     import estimate_landing_field_length
from .find_take_off_weight_given_tofl   import find_take_off_weight_given_tofl
from .generate_V_n_diagram              import generate_V_n_diagram 
from .rotor_aerodynamic_analysis        import rotor_aerodynamic_analysis  
