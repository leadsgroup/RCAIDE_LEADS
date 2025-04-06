# RCAIDE/Methods/Energy/Propulsors/Rotor_Design/__init__.py
# 

"""
Methods for designing rotors in aircraft propulsion systems.

This module provides functions for setting up and executing the design process for
various types of rotors, including propellers, lift rotors, and prop rotors. It includes
methods for blade geometry setup, optimization parameter configuration, design procedure
definition, and post-optimization parameter setting.

The design process typically involves:
    1. Setting up the blade geometry with appropriate airfoil sections and twist distribution
    2. Configuring the optimization parameters and constraints
    3. Defining the design procedure and analysis methods
    4. Running the optimization to find the optimal blade design
    5. Setting the optimized parameters in the rotor object

See Also
--------
RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance
RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Geometry
RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Noise
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .blade_geometry_setup         import blade_geometry_setup
from .optimization_setup           import optimization_setup
from .procedure_setup              import procedure_setup
from .set_optimized_parameters     import set_optimized_parameters

