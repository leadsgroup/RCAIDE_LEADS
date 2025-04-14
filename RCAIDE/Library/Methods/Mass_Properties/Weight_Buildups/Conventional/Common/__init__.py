# RCAIDE/Library/Methods/Mass_Properties/Weight_Buildups/Conventional/Transport/Common/__init__.py
# 

"""
Methods for computing component weights that are common across different aircraft configurations. 
This module provides weight estimation techniques for standard aircraft components and systems 
that share similar design principles regardless of aircraft type.

See Also
--------
RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport
    Transport-specific weight methods
RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.General_Aviation
    General aviation weight methods
RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.BWB
    BWB-specific weight methods
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .compute_payload_weight import compute_payload_weight
