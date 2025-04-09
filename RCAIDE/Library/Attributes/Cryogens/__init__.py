# RCAIDE/Library/Attributes/Cryogens/__init__.py
# 

"""
This module provides cryogenic fuel handling capabilities for RCAIDE. It includes classes for managing different types of cryogenic fuels and their properties, with specific implementations for liquid hydrogen and general cryogenic substances.

See Also
--------
RCAIDE.Library.Attributes.Propellants : Related module for conventional fuel handling
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .Cryogen         import Cryogen
from .Liquid_Hydrogen import Liquid_Hydrogen