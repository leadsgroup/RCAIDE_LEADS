"""
Provides methods for analyzing rotor performance using actuator disc theory.

This module contains implementations of the actuator disc theory for rotors, which is a simplified 
aerodynamic model that treats the rotor as an infinitely thin disc that induces a pressure 
discontinuity in the flow. The model is useful for preliminary design and analysis of rotors, 
propellers, and other rotating aerodynamic devices.

See Also
--------
RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Blade_Element_Momentum_Theory_Helmholtz_Wake
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .Actuator_Disk_performance import Actuator_Disk_performance