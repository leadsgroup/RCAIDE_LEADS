## @defgroup Methods-Geometry-Two_Dimensional/Cross_Section/Airfoil Airfoil
# RCAIDE/Methods/Geometry/Two_Dimensional/Cross_Section/Airfoil/__init__.py
# 

""" RCAIDE Package Setup
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# @ingroup Methods-Geometry-Two_Dimensional-Cross_Section

from Legacy.trunk.S.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil import compute_naca_4series 
from Legacy.trunk.S.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil import import_airfoil_dat
from Legacy.trunk.S.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil import import_airfoil_geometry 
from Legacy.trunk.S.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil import import_airfoil_polars
from Legacy.trunk.S.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil import convert_airfoil_to_meshgrid
from Legacy.trunk.S.Methods.Geometry.Three_Dimensional.estimate_naca_4_series_internal_volume     import estimate_naca_4_series_internal_volume
from .compute_airfoil_properties import compute_airfoil_properties