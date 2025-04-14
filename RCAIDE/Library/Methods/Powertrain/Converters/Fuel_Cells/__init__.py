# RCAIDE/Library/Methods/Energy/Fuel_Cells/__init__.py
# 

"""
This module provides methods for modeling and analyzing fuel cell systems in aircraft powertrains. 

It includes functionality for different fuel cell types and modeling approaches. The module 
contains specialized submodules for common fuel cell operations, Larminie-Dicks model implementation, 
and Proton Exchange Membrane (PEM) fuel cell modeling. These methods support the design, performance 
analysis, and mission simulation of fuel cell-powered aircraft.

See Also
--------
RCAIDE.Library.Components.Powertrain.Converters.Generic_Fuel_Cell_Stack
RCAIDE.Library.Methods.Powertrain.Converters
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

from . import Common
from . import Larminie_Model
from . import Proton_Exchange_Membrane