# RCAIDE/Library/Methods/Stability/__init__.py 


"""
This module provides methods for stability analysis and computation within the RCAIDE framework.

The Stability module contains several submodules implementing different approaches to stability analysis:

    - Athena_Vortex_Lattice: Implementation of Drela's AVL (Athena Vortex Lattice) method for 
    analyzing stability characteristics of aircraft configurations. 

    - Vortex_Lattice_Method: General vortex lattice method implementation for 
    analyzing lifting surfaces and complete aircraft configurations.

    - Common: Shared utilities and functions used across different stability methods,
    including coefficient transformations and atmospheric calculations.

See Also
--------
RCAIDE.Analyses.Stability
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from . import Vortex_Lattice_Method
from . import Common