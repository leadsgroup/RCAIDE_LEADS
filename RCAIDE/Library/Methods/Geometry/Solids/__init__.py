# RCAIDE/Library/Methods/Geometry/Solids/__init__.py
#

""" Volume, center of gravity and inertia of solids from their sections, on NumPy or JAX
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from .polygon_properties import polygon_moment, polygon_properties
from .cut_section        import cut_section
from .loft_properties    import loft_properties, properties_from_moments
from .combine_properties import combine_properties, mirror_properties, scale_properties
