# RCAIDE/Methods/Aerodynamics/__init__.py
# 

"""
This module provides methods for aerodynamic analysis and computation within the RCAIDE framework.

The Aerodynamics module contains several submodules implementing different approaches to aerodynamic analysis:

    - Athena_Vortex_Lattice: Implementation of Drela's AVL (Athena Vortex Lattice) method for 
    analyzing aerodynamic characteristics of aircraft configurations.

    - Airfoil_Panel_Method: Panel method implementation for 2D airfoil analysis, 
    providing pressure distributions and aerodynamic coefficients.

    - AERODAS: Implementation of the AERODAS (Aerodynamic Data Analysis System) model 
    for high angle of attack aerodynamics and stall prediction.

    - Vortex_Lattice_Method: General vortex lattice method implementation for 
    analyzing lifting surfaces and complete aircraft configurations.

    - Common: Shared utilities and functions used across different aerodynamic methods,
    including coefficient transformations and atmospheric calculations.

See Also
--------
RCAIDE.Analyses.Aerodynamics
RCAIDE.Library.Components.Wings
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from . import Athena_Vortex_Lattice
from . import Airfoil_Panel_Method 
from . import AERODAS
from . import Vortex_Lattice_Method
from . import Common