# RCAIDE/Library/Methods/Geometry/Airfoil/apply_airfoil_thickness_multiplier.py

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
from scipy import interpolate

# ----------------------------------------------------------------------------------------------------------------------
#  apply_airfoil_thickness_multiplier
# ----------------------------------------------------------------------------------------------------------------------
def apply_airfoil_thickness_multiplier(geometry, thickness_multiplier):
    """Scales an airfoil's thickness distribution about its own camber line,
    in place, leaving camber and chord untouched. Shared by
    import_airfoil_geometry() and compute_naca_4series() so both sources
    produce the same field layout under the same transform.

    Parameters
    ----------
    geometry : Data
        Airfoil geometry Data with x_upper_surface, y_upper_surface,
        x_lower_surface, y_lower_surface (shared x grid), x_coordinates,
        y_coordinates (combined-loop sampling), camber_coordinates,
        thickness_to_chord, max_thickness -- as returned by
        import_airfoil_geometry() or compute_naca_4series().
    thickness_multiplier : float
        Scale factor on local thickness (1.0 = no change).
    """
    if thickness_multiplier == 1.0:
        return geometry

    x_common  = geometry.x_upper_surface  # == x_lower_surface, shared grid
    thickness = geometry.y_upper_surface - geometry.y_lower_surface
    camber    = geometry.y_lower_surface + thickness / 2

    new_thickness = thickness * thickness_multiplier
    geometry.y_upper_surface = camber + new_thickness / 2
    geometry.y_lower_surface = camber - new_thickness / 2
    geometry.camber_coordinates = camber
    geometry.max_thickness      = geometry.max_thickness * thickness_multiplier
    geometry.thickness_to_chord = geometry.thickness_to_chord * thickness_multiplier

    # x_coordinates/y_coordinates is the same airfoil on a different (combined
    # upper+lower loop) sampling -- scale each point's deviation from the
    # local (interpolated) camber line by the same factor.
    camber_interp = interpolate.interp1d(x_common, camber, kind='linear',
                                          bounds_error=False,
                                          fill_value=(camber[0], camber[-1]))
    local_camber = camber_interp(geometry.x_coordinates)
    geometry.y_coordinates = local_camber + thickness_multiplier * (geometry.y_coordinates - local_camber)

    return geometry
