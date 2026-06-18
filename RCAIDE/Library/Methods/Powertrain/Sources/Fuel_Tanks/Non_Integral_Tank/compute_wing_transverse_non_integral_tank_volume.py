# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/Non_Integral_Tank/compute_wing_transverse_non_integral_tank_volume.py
# 
# 
# Created: Aug 2025, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import  RCAIDE
from RCAIDE.Library.Methods.Geometry.Planform.convert_sweep import convert_sweep_segments
from RCAIDE.Library.Methods.Geometry.Airfoil import import_airfoil_geometry,  compute_naca_4series
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_non_dimensional_moment_of_inertia import compute_rounded_end_cylinder_non_dimensional_moi

#Python Imports 
import numpy as np
from scipy.interpolate import interp1d
from shapely.geometry import Polygon, Point
from copy import  deepcopy
import shapely
import os

# ----------------------------------------------------------------------------------------------------------------------
#  Methods to compute volume of non integrak tanks
# ----------------------------------------------------------------------------------------------------------------------  
def compute_wing_transverse_non_integral_tank_volume(fuel_tank, wing,fuel_tanks):
    """
    Computes the volume of an aft fuel tank for a Blended Wing Body (BWB) aircraft configuration.

    This function calculates the maximum possible fuel tank volume that can fit within the aft
    section of a BWB wing, considering airfoil geometry, structural constraints, and tank dimensions.
    The tank is designed as a cylindrical tank with rounded ends positioned within the aft portion
    of the wing segments.

    Parameters
    ----------
    fuel_tank : Fuel_Tank
        Fuel tank object containing tank specifications and parameters
            - aft_tank_start_root_chord : float
                Starting position of aft tank as fraction of root chord
            - aft_tank_end_rood_chord : float
                Ending position of aft tank as fraction of root chord
            - aft_tank_end_segment_tag : str
                Tag of the wing segment where aft tank ends
            - wing_root_tag : str
                Tag of the root wing segment
            - radial_offset : float
                Radial clearance from wing structure
            - wall_thickness : float
                Thickness of tank walls
            - fuel : Fuel
                Fuel properties including density
            - orientation_euler_angles : list
                Euler angles defining tank orientation
    wing : Wing
        Wing object containing segment geometry and airfoil data
            - segments : dict
                Dictionary of wing segments with their properties
            - chords.root : float
                Root chord length
            - spans.projected : float
                Projected wing span

    Returns
    -------
    volume : float
        Maximum possible internal volume of the aft fuel tank

    Notes
    -----
    The function processes multiple wing segments to determine the optimal tank dimensions.
    It uses airfoil coordinate data to find the largest possible circular cross-section
    that fits within the wing geometry at each spanwise location.

    **Major Assumptions**
        * Tank is cylindrical with rounded ends
        * Tank is symmetric about the aircraft centerline
        * Airfoil coordinate files are available and properly formatted
        * Wing segments are properly defined with airfoil data

    **Theory**

    The tank volume is calculated as the sum of a cylindrical section and hemispherical end caps:
    
    .. math::
        V = \\pi r^2 l + \\frac{4}{3}\\pi r^3

    where r is the tank radius and l is the cylindrical length.
    """
    # Check if there are enough properties to accurately compute the maximum possible tank volume 
    if any(val is None for val in [
        fuel_tank.transverse_tank_chord_bounds[0],
        fuel_tank.transverse_tank_chord_bounds[1],
        fuel_tank.segments_bounding_tank ,
        fuel_tank.transverse_tank_segment_bound
        ]):
        raise ValueError("One or more required aft tank parameters are not set in 'fuel_tank'.")
    if hasattr( wing, 'aft_tank_end_percent'):
        if  (fuel_tank.transverse_tank_chord_bounds[1] - wing.aft_tank_end_percent - 0.005)*wing.chords.root > 0.25: # Successive Aft Tank diameter needs to be atleast 0.25m
            fuel_tank.transverse_tank_chord_bounds[0] = wing.aft_tank_end_percent + 0.005
        else:
            print(f"[WARNING] Tank '{fuel_tank.tag}' cannot not fit in the space. Removing from list.")
            fuel_tanks.pop(fuel_tank.tag)
            return
    # ------------------------------------------------------
    # compute tank bounds
    # ------------------------------------------------------
    # root chord of refernce wing
    root_chord = wing.chords.root
    wing_span  = wing.spans.projected
    
    # where tank is located as a percentage of root chord
    tank_start_percent = fuel_tank.transverse_tank_chord_bounds[0]
    tank_end_percent   = fuel_tank.transverse_tank_chord_bounds[1]

    # create x coordinates where airfoils will be interpolated to find polygon of interest
    n = 5
    # ------------------------------------------------------
    # loop through wing segments to get cooridates
    # ------------------------------------------------------
    num_tank_sections    = 0
    wing_segment_origins = np.empty((0, 3))
    segments             = wing.segments
        
    seg_tags = list(wing.segments.keys())
    index = seg_tags.index( fuel_tank.transverse_tank_segment_bound)
    seg_names = seg_tags[:index + 1]

    for _,tag in enumerate(seg_names):
        segment = wing.segments[tag]
        num_tank_sections += 1
        wing_segment_origins =  np.concatenate((wing_segment_origins, np.array(segment.origin)), axis=0)

    tank_end_percent_current = tank_end_percent
    while True:
        try:
            # dimensionalized location of tank bounds
            tank_start_dimensional = tank_start_percent * root_chord
            tank_end_dimensional   = tank_end_percent_current * root_chord
            x_tank_bounds = np.linspace(tank_start_dimensional, tank_end_dimensional, n)
            # ------------------------------------------------------
            # loop through wing segments to get cooridates
            # ------------------------------------------------------
            polygon_points = []
            for seg_i in range(num_tank_sections):
                segment = segments[seg_names[seg_i]]
                if seg_i == 0:
                    fuel_tank.wing_root_twist = segments[seg_names[seg_i]].twist
                if segment.airfoil != None:
                    if type(segment.airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                        geometry = compute_naca_4series(segment.airfoil.NACA_4_Series_code)
                    elif type(segment.airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil:
                        geometry = import_airfoil_geometry(segment.airfoil.coordinate_file)
                else:
                    geometry = compute_naca_4series('0012')
                # Get segment chord
                segment_chord = segments[seg_names[seg_i]].root_chord_percent * root_chord
                # Get upper and lower points and scale by chord
                x_points_upper = segment_chord * geometry.x_upper_surface
                x_points_lower = segment_chord * geometry.x_lower_surface
                y_points_upper = segment_chord * geometry.y_upper_surface
                y_points_lower = segment_chord * geometry.y_lower_surface
                # position points correct using segment origin (this is based on sweep and dihedral)
                x_points_upper_positioned = x_points_upper + wing_segment_origins[seg_i][0]
                x_points_lower_positioned = x_points_lower + wing_segment_origins[seg_i][0]
                y_points_upper_positioned = y_points_upper + wing_segment_origins[seg_i][2] - fuel_tank.wall_clearance
                y_points_lower_positioned = y_points_lower + wing_segment_origins[seg_i][2] + fuel_tank.wall_clearance
                # Use interpolant to evaluate polygon points
                upper_y_function = interp1d(x_points_upper_positioned, y_points_upper_positioned, kind='linear')
                lower_y_function = interp1d(x_points_lower_positioned, y_points_lower_positioned, kind='linear')
                upper_y_raw      = upper_y_function(x_tank_bounds)
                lower_y_raw      = lower_y_function(x_tank_bounds)
                # Reflexed airfoils can have upper_y < lower_y near the trailing edge;
                # clamp so the polygon is always non-self-intersecting.
                upper_y_points   = np.maximum(upper_y_raw, lower_y_raw)
                lower_y_points   = np.minimum(upper_y_raw, lower_y_raw)

                # Create polygon
                polygon = []
                # upper points
                idx = 0
                for polygon_corner in range(n):
                    polygon.append((x_tank_bounds[polygon_corner], upper_y_points[polygon_corner]))
                    idx += 1
                # lower points
                for polygon_corner_rev in range(n-1, -1, -1):
                    polygon.append((x_tank_bounds[polygon_corner_rev], lower_y_points[polygon_corner_rev]))
                    idx += 1
                # close polygon
                polygon.append((x_tank_bounds[0], upper_y_points[0]))
                # store polygon points
                polygon_points.append(polygon)
                # test polygon
                # Extract x and y coordinates into separate lists for plotting
                x_coords = [p[0] for p in polygon_points[seg_i]]
                y_coords = [p[1] for p in polygon_points[seg_i]]
            fuel_tank.transverse_tank_chord_bounds[1] = tank_end_percent_current
            break
        except Exception:
            tank_end_percent_current -= 0.01
            if tank_end_percent_current <= tank_start_percent:
                raise ValueError("Unable to compute aft tank polygon bounds after reducing transverse_tank_chord_bounds[1].")
    # ------------------------------------------------------------------------------------------------------
    # Iteratively get maximum inscribed circle between wing segment circles and store volume
    # ------------------------------------------------------------------------------------------------------
    tank_radii     = np.zeros(num_tank_sections-1)
    tank_volumes   = np.zeros(num_tank_sections-1)
    tank_lengths   = np.zeros(num_tank_sections-1)
    circle_origins = np.zeros((num_tank_sections-1, 2))
    for seg_i in  range(1,num_tank_sections):
        if seg_i == 1:
            inner_polygon = Polygon(polygon_points[seg_i - 1]).buffer(0)
        else:
            inner_polygon = intersection_polygon
        outer_polygon = Polygon(polygon_points[seg_i]).buffer(0)
        # intersection polygon
        intersection_polygon = inner_polygon.intersection(outer_polygon)
        if intersection_polygon.is_empty:
            continue
        if not isinstance(intersection_polygon, Polygon):
            polys = [g for g in getattr(intersection_polygon, 'geoms', []) if isinstance(g, Polygon)]
            intersection_polygon = max(polys, key=lambda g: g.area) if polys else None
        if intersection_polygon is None:
            continue
        # maximum radius
        poly             = Polygon(intersection_polygon)
        inscribed_circle =  shapely.maximum_inscribed_circle(poly)
        circle_center_x  =  inscribed_circle.coords[0][0]
        circle_center_y  =  inscribed_circle.coords[0][1]
        boundary_x       =  inscribed_circle.coords[1][0]
        boundary_y       = inscribed_circle.coords[1][1]
        tank_radius      =  np.sqrt( (boundary_x - circle_center_x) ** 2 + (boundary_y - circle_center_y) ** 2 )
        # store radius
        tank_radii[seg_i-1] = tank_radius
        # compute and store volume
        l_total                 =  segments[seg_names[seg_i]].percent_span_location *  wing_span
        height                  = l_total - 2 *tank_radius
        volume                  = 4/3 *np.pi * (tank_radius ** 3) +   np.pi * (tank_radius ** 2) *  height
        tank_volumes[seg_i-1]   =  volume
        tank_lengths[seg_i-1]   = height
        circle_origins[seg_i-1][0] = circle_center_x
        circle_origins[seg_i-1][1] = circle_center_y
    # ------------------------------------------------------------------------------------------------------
    # Get Maximum volume and corresponding properties
    # ------------------------------------------------------------------------------------------------------
    max_volume = np.max(tank_volumes)
    max_idx    = np.argmax(tank_volumes)
    radius_opt = tank_radii[max_idx]
    length_opt = tank_lengths[max_idx]

    # lengths.external is total tip-to-tip (cylinder + two hemispheres)
    fuel_tank.diameters.external = radius_opt * 2
    fuel_tank.lengths.external   = length_opt + fuel_tank.diameters.external
    fuel_tank.diameters.internal = radius_opt * 2
    fuel_tank.aspect_ratio       = fuel_tank.lengths.external / fuel_tank.diameters.external

    fuel_tank.lengths.internal   = fuel_tank.aspect_ratio * fuel_tank.diameters.internal
    fuel_tank.volume_properties.net_volume   = max_volume
    fuel_tank.volume_properties.gross_volume = max_volume

    # fuel tank origin
    fuel_tank.origin[0][0] = circle_origins[max_idx][0] - fuel_tank.diameters.external / 2
    fuel_tank.origin[0][1] = 0
    fuel_tank.origin[0][2] = circle_origins[max_idx][1]

    # fuel tank C.G.
    fuel_tank.fuel.mass_properties.center_of_gravity = [[fuel_tank.lengths.external / 2, 0, 0]]
    fuel_tank.mass_properties.center_of_gravity      = [[fuel_tank.lengths.external / 2, 0, 0]]
    if fuel_tank.orientation_euler_angles == [0., 0., np.pi/2]:
        fuel_tank.fuel.mass_properties.center_of_gravity = [[fuel_tank.diameters.external / 2, 0, 0]]
        fuel_tank.mass_properties.center_of_gravity      = [[fuel_tank.diameters.external / 2, 0, 0]]

    fuel_tank.fuel.origin             = fuel_tank.origin
    fuel_tank.fuel.xz_plane_symmetric = wing.xz_plane_symmetric
    fuel_tank.fuel.xy_plane_symmetric = wing.xy_plane_symmetric
    fuel_tank.fuel.yz_plane_symmetric = wing.yz_plane_symmetric
    wing.aft_tank_end_percent         = (fuel_tank.origin[0][0] + fuel_tank.diameters.external) / wing.chords.root

    # MOI helper expects cylinder-only length
    L_cyl_i = fuel_tank.lengths.internal - fuel_tank.diameters.internal
    fuel_tank.fuel.mass_properties.moments_of_inertia.non_dimensional_tensor = compute_rounded_end_cylinder_non_dimensional_moi(fuel_tank.diameters.internal / 2, L_cyl_i)

    return


def compute_largest_circle(x_points, z_upper, z_lower):
    """
    Computes the largest circle that can fit within a polygon defined by airfoil coordinates.

    This function finds the optimal center point and radius for the largest possible circle
    that fits within the polygon formed by the upper and lower airfoil surfaces. It uses
    a grid search approach to find the best center location.

    Parameters
    ----------
    x_points : array_like
        X-coordinates of the airfoil points
    z_upper : array_like
        Z-coordinates of the upper airfoil surface
    z_lower : array_like
        Z-coordinates of the lower airfoil surface

    Returns
    -------
    max_diameter : float
        Diameter of the largest possible circle
    x_center : float
        X-coordinate of the circle center
    z_center : float
        Z-coordinate of the circle center

    Notes
    -----
    The function creates a polygon from the airfoil coordinates and performs a grid search
    within the polygon's bounding box to find the optimal circle center. The radius is
    limited by the distance to the closest polygon edge.

    **Major Assumptions**
        * Airfoil coordinates form a valid polygon
        * Grid resolution is sufficient for accurate results
        * Polygon is simply connected

    **Theory**

    The largest circle is found by maximizing the radius r such that:
    
    .. math::
        r = \\min_{i} d(p, e_i)

    where p is the circle center and e_i are the polygon edges.

    **Definitions**

    'Inscribed Circle'
        The largest circle that can fit completely within a given polygon
    """

    coords = list(zip(x_points, z_upper)) + list(zip(x_points[::-1], z_lower[::-1]))
    poly   = Polygon(coords)

    #scan a fine grid inside the polygon's bounding box to find the best center
    minx, minz, maxx, maxz = poly.bounds
    nx, nz = 200, 200  
    xs = np.linspace(minx, maxx, nx)
    zs = np.linspace(minz, maxz, nz)

    best_r = 0.0
    best_pt = None

    for x in xs:
        for z in zs:
            p = Point(x, z)
            if not poly.contains(p):
                continue
            # the radius is limited by the closest polygon edge
            r = p.distance(poly.exterior)
            if r > best_r:
                best_r = r
                best_pt = (x, z)
    max_diameter = 2 * best_r
    
    return max_diameter , best_pt[0],best_pt[1]
