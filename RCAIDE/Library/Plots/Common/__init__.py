## @defgroup Library-Plots-Performance-Common Common 
# RCAIDE/Library/Plots/Performance/Common/__init__.py
# 

"""
RCAIDE Common Plotting Utilities

This module provides standardized plotting utilities used across RCAIDE's visualization 
capabilities. It includes functions for consistent axis formatting and plot styling.
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
 
from .set_axes            import set_axes
from .plot_style          import plot_style
from .rcaide_colormap     import segment_colors, _RCAIDE_CMAP