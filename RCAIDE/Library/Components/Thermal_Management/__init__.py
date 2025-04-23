# RCAIDE/Energy/Thermal_Management/__init__.py

"""
Module containing thermal management components for aircraft systems. This module provides 
classes for modeling heat transfer and thermal control through various components including 
heat exchangers, batteries, and coolant reservoirs.

Notes
-----
The thermal management system components are organized into several submodules:

* Accessories: Support components for thermal systems
* Batteries: Thermal management for energy storage systems
* Heat_Exchangers: Components for heat transfer and rejection
* Reservoirs: Coolant and thermal storage components

See Also
--------
RCAIDE.Library.Components.Energy_Systems
    Related module for energy system thermal management
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from . import Accessories
from . import Batteries
from . import Heat_Exchangers
from . import Reservoirs

