"""
Methods for rotor performance analysis using Blade Element Momentum Theory with Helmholtz wake modeling.

This module provides functions for analyzing rotor performance using an advanced implementation
of Blade Element Momentum Theory (BEMT) that incorporates Helmholtz wake modeling. This approach
combines the computational efficiency of BEMT with a more physically accurate representation of
the wake structure using Helmholtz vortex filaments.

The Helmholtz wake model accounts for the helical vortex structure shed from the rotor blades,
providing improved predictions of induced velocities, especially in forward flight and for
highly loaded rotors. The module includes methods for computing wake-induced velocities,
wake contraction, and overall rotor performance metrics such as thrust, torque, and power.

See Also
--------
RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Blade_Element_Momentum_Theory
RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Actuator_Disc_Theory
RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Geometry
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .BEMT_Helmholtz_performance               import BEMT_Helmholtz_performance
from .compute_wake_induced_velocity            import compute_wake_induced_velocity 
from .compute_wake_contraction_matrix          import compute_wake_contraction_matrix 
from .wake_model                               import *
