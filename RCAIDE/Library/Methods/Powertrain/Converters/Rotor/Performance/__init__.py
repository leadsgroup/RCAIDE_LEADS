# Rotor_Wake provides the functions needed to perform analyses.

"""
Methods for analyzing and computing the performance of rotors in aircraft propulsion systems.

This module provides functions for calculating the aerodynamic performance of various types
of rotors, including propellers, lift rotors, and prop rotors. It implements different
analysis methods such as Blade Element Momentum Theory (BEMT) with Helmholtz wake modeling
and Actuator Disc Theory.

The performance analysis methods calculate key parameters such as thrust, torque, power,
efficiency, and induced velocities based on the rotor geometry, operating conditions, and
flight state. These methods are essential for predicting the behavior of rotors in different
flight regimes, from hover to forward flight.

See Also
--------
RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Design
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# -------------------------------------------------------------------------------------------------------------------- 
from . import Blade_Element_Momentum_Theory_Helmholtz_Wake
from . import Actuator_Disc_Theory






