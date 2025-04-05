# RCAIDE/Energy/Thermal_Management/Batteries/__init__.py

"""
Module containing thermal management components specifically for battery systems. This module 
provides classes for different battery cooling approaches including air cooling, cryogenic 
cooling, and liquid cooling with wavy channels.

See Also
--------
RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules
    Related module for battery system modeling
RCAIDE.Library.Components.Thermal_Management.Heat_Exchangers
    Related module for heat exchanger components used in battery cooling
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from .Air_Cooled                 import Air_Cooled
from .Liquid_Cooled_Wavy_Channel import Liquid_Cooled_Wavy_Channel