# RCAIDE/Methods/Mission/Common/Residuals/__init__.py
# 

"""
This module contains (a) residual functions used in mission analysis for RCAIDE. It provides 
flight dynamics residual calculations that are essential for solving flight dynamics 
equations during mission simulation.

See Also
--------
RCAIDE.Library.Mission.Segments : Mission segment definitions that may use these residuals
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# -----------------------------------------------------------------------------------------------------------------------Common
   
from .flight_dynamics import flight_dynamics  