# RCAIDE/Methods/Aerodynamics/Common/Lift/__init__.py
# 

"""
Common RCAIDE Lift Module Aerodynamic Methods

This module provides lift calculation functions and methods used across multiple analyses.

See Also
--------
RCAIDE.Library.Methods.Aerodynamics.Common.Drag
RCAIDE.Framework.Analyses.Aerodynamics
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 


from .compute_max_lift_coeff   import compute_max_lift_coeff
from .compute_flap_lift        import compute_flap_lift
from .compute_slat_lift        import compute_slat_lift 
from .BET_calculations         import compute_airfoil_aerodynamics 
from .BET_calculations         import compute_inflow_and_tip_loss
from .fuselage_correction      import fuselage_correction