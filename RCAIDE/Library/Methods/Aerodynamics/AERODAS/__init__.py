# RCAIDE/Methods/Aerodynamics/AERODAS/__init__.py
# 

"""
AERODAS aerodynamic coefficient computation methods for airfoils from NASA contractor report CR-2008-215434, October 2008. 
"Models of Lift and Drag Coefficients of Stalled and Unstalled Airfoils in Wind Turbines and Wind Tunnels"
by David A. Spera.

This module provides empirical methods for computing lift and drag coefficients
in both pre-stall and post-stall flow regimes. The AERODAS approach uses 
geometric parameters and flight conditions to predict section aerodynamic 
characteristics. The model was originally developed for wind turbine and wind tunnel applications.

See Also
--------
RCAIDE.Library.Methods.Aerodynamics : Parent aerodynamics methods module
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
  
from .post_stall_coefficients import post_stall_coefficients
from .pre_stall_coefficients  import pre_stall_coefficients