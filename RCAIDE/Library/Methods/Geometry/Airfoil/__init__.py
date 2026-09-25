# RCAIDE/Methods/Geometry/Two_Dimensional/Cross_Section/Airfoil/__init__.py
# 

""" RCAIDE Package Setup
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .compute_naca_4series               import compute_naca_4series
from .compute_airfoil_properties         import compute_airfoil_properties
from .import_airfoil_dat                 import import_airfoil_dat
from .import_airfoil_geometry            import import_airfoil_geometry
from .import_airfoil_polars              import import_airfoil_polars
from .convert_airfoil_to_meshgrid        import convert_airfoil_to_meshgrid
from .apply_airfoil_thickness_multiplier import apply_airfoil_thickness_multiplier