# RCAIDE/Library/Attributes/Coolants/__init__.py
# 

"""
RCAIDE Coolants module provides models and properties for various cooling fluids used in aircraft systems.

These models provide essential thermodynamic and physical properties needed for heat 
transfer calculations and thermal management system analysis, including Specific heat capacity,
Thermal conductivity, Density, Viscosity, and Heat transfer coefficients.

See Also
--------
RCAIDE.Library.Attributes.Cryogens : Cryogenic fluid properties
RCAIDE.Library.Components.Energy : Energy system components
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
 
from .Coolant       import Coolant
from .Glycol_Water  import Glycol_Water
