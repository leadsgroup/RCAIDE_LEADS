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
from .aircraft_aerodynamic_analysis         import aircraft_aerodynamic_analysis
from .compute_load_and_trim_diagram         import compute_load_and_trim_diagram
from .compute_noise_certification_metrics   import compute_noise_certification_metrics
from .compute_payload_range_diagram         import compute_payload_range_diagram
from .compute_V_n_diagram                   import compute_V_n_diagram 
from .estimate_landing_field_length         import estimate_landing_field_length
from .estimate_stall_speed                  import estimate_stall_speed
from .estimate_take_off_field_length        import estimate_take_off_field_length
from .estimate_take_off_weight_given_TOFL   import estimate_take_off_weight_given_TOFL
from .generate_cruise_drag_buildup_table    import generate_cruise_drag_buildup_table
from .rotor_aerodynamic_analysis            import rotor_aerodynamic_analysis  
