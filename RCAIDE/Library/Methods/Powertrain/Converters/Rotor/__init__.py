# RCAIDE/Methods/Energy/Propulsors/Converters/Rotor/__init__.py
# 

"""
Methods for analyzing and designing rotors in aircraft propulsion systems.

This module provides a comprehensive set of functions for the design, analysis, and
performance evaluation of various types of rotors, including propellers, lift rotors,
and prop rotors. It implements different analysis methods such as Blade Element Momentum
Theory (BEMT) with Helmholtz wake modeling and Actuator Disc Theory.

The module includes functions for:
    - Designing optimal rotor geometries for different applications
    - Computing rotor performance across various flight conditions
    - Analyzing wake interactions and induced velocities
    - Setting up rotor operating conditions for mission analysis

These methods support both conventional and advanced aircraft configurations, including
fixed-wing aircraft, helicopters, multicopters, and hybrid VTOL concepts.

See Also
--------
RCAIDE.Library.Components.Powertrain.Converters.Rotor
RCAIDE.Library.Methods.Aerodynamics.Common.Lift
RCAIDE.Library.Methods.Geometry.Airfoil
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
 
from .                          import Design
from .                          import Performance
from .append_rotor_conditions   import append_rotor_conditions
from .compute_rotor_performance import compute_rotor_performance
from .design_propeller          import design_propeller 
from .design_lift_rotor         import design_lift_rotor
from .design_prop_rotor         import design_prop_rotor

