"""
Methods for rotor performance analysis using lifting-line theory with
Biot-Savart bound-vortex induction.

This module provides functions for analyzing rotor performance using a
lifting-line method, where the blade is represented as a series of bound
vortex segments along the 1/4-chord line. Induced velocities are computed
directly via the Biot-Savart law at 3/4-chord collocation points, avoiding
the inflow/circulation closure assumptions of Blade Element Momentum Theory.

See Also
--------
RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Blade_Element_Momentum_Theory_Helmholtz_Wake
"""
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .initialize_lifting_line             import initialize_lifting_line
from .initialize_wake_geometry            import initialize_wake_geometry
from .biot_savart_velocity_induction      import biot_savart_velocity_induction
from .free_wake                           import free_wake
from .lifting_line_performance            import lifting_line_performance
from .evaluate_bound_vortex_circulation   import evaluate_bound_vortex_circulation
from .compute_lifting_line_loads          import compute_lifting_line_loads